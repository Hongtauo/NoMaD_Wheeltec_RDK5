"""Independent preview-file/GUI lifecycle tests; no robot, ROS or GPU required."""
import base64
import io
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from types import ModuleType
import unittest
from unittest import mock
import uuid

import campus_preview_snapshot as snapshot
import run_campus_api
import run_campus_viewer


def prediction(stamp=None):
    return dict(schema=1, writer_id=str(uuid.uuid4()), published_ns=stamp or time.time_ns(), kind='prediction',
                frame={'stamp_ns': 123, 'jpeg_b64': base64.b64encode(b'\xff\xd8test\xff\xd9').decode()},
                prediction={'sequence': 1, 'inference_ms': 140., 'waypoint': [0.1, 0.],
                            'trajectories': [[[0.1, 0.] for _ in range(8)] for _ in range(8)]})


class SnapshotValidationTests(unittest.TestCase):
    def test_only_public_fields_are_permitted(self):
        data = prediction()
        self.assertIs(snapshot.validate_snapshot(data), data)
        for scope in (data, data['frame'], data['prediction']):
            scope['token'] = 'not-allowed'
            with self.assertRaises(ValueError):
                snapshot.validate_snapshot(data)
            del scope['token']

    def test_invalid_preview_values_are_rejected(self):
        for key, value in [('sequence', True), ('inference_ms', float('nan')),
                           ('waypoint', [float('inf'), 0]), ('trajectories', [])]:
            data = prediction()
            data['prediction'][key] = value
            with self.assertRaises(ValueError):
                snapshot.validate_snapshot(data)
        data = prediction()
        data['frame']['jpeg_b64'] = 'x' * (snapshot.MAX_SNAPSHOT_BYTES + 1)
        with self.assertRaises(ValueError):
            snapshot.validate_snapshot(data)

    def test_reader_never_replays_stale_or_future_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'latest.json'
            data = prediction(10000000000)
            path.write_text(json.dumps(data))
            reader = snapshot.SnapshotReader(path)
            self.assertEqual(reader.latest(now_ns=10500000000), data)
            self.assertIsNone(reader.latest(now_ns=11000000001))
            self.assertIsNone(reader.latest(now_ns=9000000000))
            path.unlink()
            self.assertIsNone(reader.latest(now_ns=10500000000))

    def test_oversized_file_is_bounded(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'latest.json'
            path.write_bytes(b'x' * (snapshot.MAX_SNAPSHOT_BYTES + 1))
            with self.assertRaises(ValueError):
                snapshot.SnapshotReader(path).latest()

    def test_submit_has_one_slot_and_no_io_or_credential_copy(self):
        writer = snapshot.SnapshotWriter.__new__(snapshot.SnapshotWriter)
        writer.writer_id, writer._closing = str(uuid.uuid4()), False
        writer._condition, writer.error = threading.Condition(), None
        data = prediction()
        request = {'frames': [data['frame']], 'token': 'secret', 'telemetry': {'secret': 'x'}}
        response = dict(data['prediction'], token='secret')
        with mock.patch.object(snapshot.os, 'open', side_effect=AssertionError('disk in callback')):
            for seq in range(1, 100):
                response['sequence'] = seq
                writer.submit(request, response, None)
        self.assertEqual(writer._pending['prediction']['sequence'], 99)
        self.assertNotIn('secret', json.dumps(writer._pending))
        self.assertEqual(set(writer._pending['prediction']), {'sequence', 'inference_ms', 'waypoint', 'trajectories'})

    def test_malformed_callback_never_raises_to_inference(self):
        writer = snapshot.SnapshotWriter.__new__(snapshot.SnapshotWriter)
        writer.writer_id, writer.error = str(uuid.uuid4()), None
        writer.submit({}, {}, None)
        self.assertEqual(writer.error, 'KeyError')

    def test_preview_raw_image_decode(self):
        try:
            from PIL import Image
        except ImportError:
            self.skipTest('Pillow is not installed in this host test runtime')
        from campus_rviz_bridge import decode_preview_image
        stream = io.BytesIO()
        Image.new('RGB', (32, 24), (20, 40, 60)).save(stream, format='JPEG')
        width, height, pixels = decode_preview_image(base64.b64encode(stream.getvalue()).decode())
        self.assertEqual((width, height, len(pixels)), (32, 24, 32 * 24 * 3))


@unittest.skipUnless(os.name == 'posix', 'Linux atomic/permission/directory-lock semantics')
class SnapshotLinuxTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / 'preview' / 'latest.json'
        patch = mock.patch.object(snapshot, 'PREVIEW_PATH', self.path)
        patch.start()
        self.addCleanup(patch.stop)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            try:
                if predicate():
                    return
            except (OSError, ValueError):
                pass
            time.sleep(.005)
        self.fail('Preview writer did not reach expected state')

    def test_atomic_latest_only_mode_600_and_own_cleanup(self):
        writer = snapshot.SnapshotWriter(self.path)
        try:
            data = prediction()
            writer.submit({'frames': [data['frame']]}, data['prediction'], None)
            self.wait_for(lambda: json.loads(self.path.read_text()).get('kind') == 'prediction')
            self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(len(list(self.path.parent.iterdir())), 1)
        finally:
            self.assertTrue(writer.close())
        self.assertFalse(self.path.exists())
        self.assertEqual(list(self.path.parent.iterdir()), [])

    def test_old_writer_cannot_remove_new_session_snapshot(self):
        first = snapshot.SnapshotWriter(self.path)
        second = None
        try:
            self.wait_for(lambda: json.loads(self.path.read_text()).get('writer_id') == first.writer_id)
            second = snapshot.SnapshotWriter(self.path)
            self.wait_for(lambda: json.loads(self.path.read_text()).get('writer_id') == second.writer_id)
            self.assertTrue(first.close())
            self.assertEqual(json.loads(self.path.read_text())['writer_id'], second.writer_id)
        finally:
            first.close()
            if second:
                second.close()

    def test_paths_outside_dedicated_preview_are_rejected(self):
        with self.assertRaises(ValueError):
            snapshot.SnapshotWriter(Path(self.folder.name) / 'elsewhere.json')

    def test_writer_io_error_is_isolated(self):
        with mock.patch.object(snapshot.SnapshotWriter, '_write', side_effect=PermissionError('test')):
            writer = snapshot.SnapshotWriter(self.path)
            try:
                self.wait_for(lambda: writer.error == 'PermissionError')
                data = prediction()
                writer.submit({'frames': [data['frame']]}, data['prediction'], None)
                self.assertTrue(writer._thread.is_alive())
            finally:
                writer.close()

    def test_reader_rejects_fifo_and_symlink(self):
        self.path.parent.mkdir()
        fifo = self.path.parent / 'fifo'
        os.mkfifo(fifo)
        with self.assertRaises(ValueError):
            snapshot.SnapshotReader(fifo).latest()
        target = self.path.parent / 'real.json'
        target.write_text(json.dumps(prediction()))
        self.path.symlink_to(target)
        with self.assertRaises(OSError):
            snapshot.SnapshotReader(self.path).latest()

    def test_unsafe_temp_hardlink_is_never_truncated(self):
        self.path.parent.mkdir()
        target = self.path.parent / 'unrelated'
        target.write_bytes(b'preserve-me')
        os.link(target, self.path.with_name('.latest.json.tmp'))
        writer = snapshot.SnapshotWriter(self.path)
        try:
            self.wait_for(lambda: writer.error == 'ValueError')
            self.assertEqual(target.read_bytes(), b'preserve-me')
        finally:
            writer.close()


class IndependentLifecycleTests(unittest.TestCase):
    def fake_server(self):
        module = ModuleType('nomad_navigation.campus_api_server')
        module.main = mock.Mock(return_value=0)
        return module

    def test_legacy_rviz_option_never_spawns_gui_or_installs_kill_supervisor(self):
        server = self.fake_server()
        writer = mock.Mock()
        with mock.patch.dict(sys.modules, {'nomad_navigation.campus_api_server': server}), \
                mock.patch.object(snapshot, 'SnapshotWriter', return_value=writer), \
                mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('server spawning GUI')):
            self.assertEqual(run_campus_api.main(['--rviz', '--device', 'cuda']), 0)
        server.main.assert_called_once_with(['--device', 'cuda'], on_result=writer.submit)
        writer.close.assert_called_once()

    def test_missing_preview_filesystem_does_not_prevent_inference(self):
        server = self.fake_server()
        with mock.patch.dict(sys.modules, {'nomad_navigation.campus_api_server': server}), \
                mock.patch.object(snapshot, 'SnapshotWriter', side_effect=PermissionError('test')):
            self.assertEqual(run_campus_api.main(['--device', 'cuda']), 0)
        server.main.assert_called_once_with(['--device', 'cuda'], on_result=None)

    def test_viewer_quit_only_closes_local_preview(self):
        bridge = ModuleType('campus_rviz_bridge')
        preview = mock.Mock(error=None)
        bridge.CampusPreview = mock.Mock(return_value=preview)
        rviz = mock.Mock()
        rviz.poll.return_value = 0
        with mock.patch.dict(sys.modules, {'campus_rviz_bridge': bridge}), \
                mock.patch.dict(os.environ, ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1'), \
                mock.patch.object(subprocess, 'Popen', return_value=rviz), \
                mock.patch.object(signal, 'signal', return_value=None):
            self.assertEqual(run_campus_viewer.main(['--snapshot-file', str(Path.cwd() / 'latest.json')]), 0)
        preview.close.assert_called_once()
        rviz.send_signal.assert_not_called()

    def test_viewer_rejects_robot_dds_domain_before_creating_processes(self):
        with mock.patch.dict(os.environ, ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='1'), \
                mock.patch.object(subprocess, 'Popen') as popen:
            with self.assertRaises(ValueError):
                run_campus_viewer.main([])
        popen.assert_not_called()

    def test_viewer_mounts_no_credentials_and_has_no_network(self):
        project = Path(__file__).resolve().parents[2]
        script = (project / 'docker/ros2/start_campus_rviz.ps1').read_text(encoding='utf-8')
        self.assertIn("'--network','none'", script)
        self.assertNotIn('dst=/credentials', script)
        self.assertNotIn('--gpus', script)
        self.assertIn('type=volume,src=$previewVolume,dst=/preview,readonly,volume-nocopy', script)
        self.assertNotIn('$previewDirectory', script)
        self.assertIn('Get-CampusPreviewVolume', script)
        server_script = (project / 'docker/ros2/start_campus_api.ps1').read_text(encoding='utf-8')
        self.assertIn('Get-CampusPreviewVolume -CreateIfMissing', server_script)
        self.assertIn('type=volume,src=$previewVolume,dst=/workspace/SocialEmbodiedAgent/outputs/campus_api/preview,volume-nocopy', server_script)
        self.assertNotIn('$previewDirectory', server_script)
        config = (project / 'ros2_ws/src/nomad_navigation/rviz/nomad_campus.rviz').read_text()
        self.assertIn('SERVER PREDICTIONS ONLY', config)
        self.assertIn('Transport Hint: raw', config)
        self.assertIn('/nomad/campus_preview/image', config)

    def test_powershell_tmpfs_volume_metadata_conflicts_are_refused(self):
        shell = shutil.which('pwsh') or shutil.which('powershell')
        if not shell:
            self.skipTest('PowerShell unavailable; no real Docker calls are made by this test')
        project = Path(__file__).resolve().parents[2]
        helper = str(project / 'docker/ros2/campus_preview_volume.ps1').replace("'", "''")
        command = r'''
$ErrorActionPreference = 'Stop'
. '__HELPER__'
$base = [pscustomobject]@{Name='socialembodiedagent-campus-preview'; Driver='local'; Scope='local';
    Labels=@{'org.socialembodiedagent.role'='campus-preview-tmpfs'};
    Options=@{type='tmpfs'; device='tmpfs'; o='size=4m,mode=0700'}}
function Reset-TestVolume {
    $script:Volume = $base | ConvertTo-Json -Depth 8 | ConvertFrom-Json
    $script:Exists = $true
    $script:Calls = [Collections.Generic.List[string]]::new()
}
function docker {
    $script:Calls.Add(($args -join ' '))
    $global:LASTEXITCODE = 0
    if ($args[0] -ne 'volume') { throw 'Unexpected real Docker operation' }
    switch ($args[1]) {
        'ls' { if ($script:Exists) { 'socialembodiedagent-campus-preview' } }
        'create' { $script:Exists = $true; 'socialembodiedagent-campus-preview' }
        'inspect' { $script:Volume | ConvertTo-Json -Depth 8 }
        default { throw 'Unexpected mutating Docker operation' }
    }
}
Reset-TestVolume
if ((Get-CampusPreviewVolume) -ne $base.Name) { throw 'Valid existing volume rejected' }
if ($script:Calls.Count -ne 2) { throw 'Existing volume was unexpectedly changed' }
Reset-TestVolume
$script:Exists = $false
$rejected = $false
try { Get-CampusPreviewVolume | Out-Null } catch { $rejected = $true }
if (-not $rejected -or $script:Calls.Count -ne 1) { throw 'Viewer created missing volume' }
Reset-TestVolume
$script:Exists = $false
if ((Get-CampusPreviewVolume -CreateIfMissing) -ne $base.Name) { throw 'Creation rejected' }
if ($script:Calls.Count -ne 3 -or $script:Calls[1] -notmatch 'type=tmpfs.*device=tmpfs.*o=size=4m,mode=0700') {
    throw 'Creation options differ from required tmpfs'
}
$mutations = @(
    { $script:Volume.Name = 'wrong' },
    { $script:Volume.Driver = 'other' },
    { $script:Volume.Scope = 'global' },
    { $script:Volume.Labels.'org.socialembodiedagent.role' = 'other' },
    { $script:Volume.Labels = $null },
    { $script:Volume.Options.type = 'ext4' },
    { $script:Volume.Options.device = '/disk' },
    { $script:Volume.Options.o = 'size=8m,mode=0700' },
    { $script:Volume.Options.o = 'size=4m,mode=0777' },
    { $script:Volume.Options = $null },
    { $script:Volume.Options | Add-Member -NotePropertyName extra -NotePropertyValue unsafe }
)
foreach ($mutation in $mutations) {
    Reset-TestVolume
    & $mutation
    $rejected = $false
    try { Get-CampusPreviewVolume -CreateIfMissing | Out-Null } catch { $rejected = $true }
    if (-not $rejected -or $script:Calls.Count -ne 2) { throw 'Conflicting volume was accepted or mutated' }
}
# A concurrent creator can win the name between ls and create; inspect must still reject it.
Reset-TestVolume
$script:Exists = $false
$script:Volume.Options.type = 'ext4'
$rejected = $false
try { Get-CampusPreviewVolume -CreateIfMissing | Out-Null } catch { $rejected = $true }
if (-not $rejected -or $script:Calls.Count -ne 3) { throw 'Creation race was not rechecked' }
$scripts = @('campus_preview_volume.ps1','start_campus_api.ps1','start_campus_rviz.ps1')
foreach ($scriptName in $scripts) {
    $tokens = $null; $parseErrors = $null
    [Management.Automation.Language.Parser]::ParseFile((Join-Path (Split-Path '__HELPER__') $scriptName), [ref]$tokens, [ref]$parseErrors) | Out-Null
    if ($parseErrors.Count) { throw 'PowerShell parse failure' }
}
'TMPFS MOCK TESTS PASSED'
'''.replace('__HELPER__', helper)
        result = subprocess.run([shell, '-NoProfile', '-NonInteractive', '-Command', command],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('TMPFS MOCK TESTS PASSED', result.stdout)


if __name__ == '__main__':
    unittest.main()
