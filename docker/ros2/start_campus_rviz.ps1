# Independent GUI: only preview/code mounts, no API token/key, no campus network.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Split-Path (Split-Path $PSScriptRoot -Parent) -Parent))
. (Join-Path $PSScriptRoot 'campus_preview_volume.ps1')
$previewVolume = Get-CampusPreviewVolume
$viewerName = 'nomad-campus-rviz'
$imageName = 'socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909'
$names = @(& docker ps -a --format '{{.Names}}')
if ($LASTEXITCODE -ne 0) { throw 'Docker is unavailable.' }
if ($viewerName -in $names) { throw 'nomad-campus-rviz already exists; inspect it before starting another viewer.' }
$arguments = @('run','--rm','-d','--pull','never','--init','--name',$viewerName,
    '--label','org.socialembodiedagent.role=campus-prediction-viewer',
    '--network','none','--shm-size=256m',
    '--mount',"type=bind,src=$(Join-Path $projectRoot 'ros2_ws'),dst=/workspace/SocialEmbodiedAgent/ros2_ws,readonly",
    '--mount',"type=volume,src=$previewVolume,dst=/preview,readonly,volume-nocopy",
    '--mount','type=bind,src=/mnt/host/wslg/.X11-unix,dst=/tmp/.X11-unix,readonly',
    '-e','DISPLAY=:0','-e','QT_X11_NO_MITSHM=1','-e','LIBGL_ALWAYS_SOFTWARE=1',
    '-e','ROS_DOMAIN_ID=87','-e','ROS_LOCALHOST_ONLY=1','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
    '-e','TZ=Asia/Shanghai',
    '-e','PYTHONPATH=/workspace/SocialEmbodiedAgent/ros2_ws/src/nomad_navigation',
    $imageName,'bash','-lc',
    'unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE ROS_DISCOVERY_SERVER RMW_FASTRTPS_USE_QOS_FROM_XML; source /opt/ros/humble/setup.bash && exec python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/run_campus_viewer.py --snapshot-file /preview/latest.json')
& docker @arguments
if ($LASTEXITCODE -ne 0) { throw 'Viewer creation failed. The inference service was not changed.' }
Write-Host 'Independent RViz requested; inspect: docker logs nomad-campus-rviz'
Write-Host 'Closing RViz removes only its viewer container; the API stays running.'
Write-Host 'Display: SERVER PREDICTIONS ONLY; client acceptance and actual motion remain unknown.'
