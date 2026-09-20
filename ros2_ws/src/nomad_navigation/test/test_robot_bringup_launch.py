"""Exercise launch ordering with stand-in actions, never starting a process."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)


def load_launch(monkeypatch, profile='0.2', allow_token=True, stub_native=True, network_pause_resume='false'):
    classes = {name: type(name, (Action,), {}) for name in (
        'LaunchDescription', 'DeclareLaunchArgument', 'EmitEvent', 'ExecuteProcess',
        'LogInfo', 'OpaqueFunction', 'RegisterEventHandler', 'OnProcessExit', 'Shutdown')}
    modules = {
        'launch': ('LaunchDescription',),
        'launch.actions': ('DeclareLaunchArgument', 'EmitEvent', 'ExecuteProcess',
                           'LogInfo', 'OpaqueFunction', 'RegisterEventHandler'),
        'launch.event_handlers': ('OnProcessExit',),
        'launch.events': ('Shutdown',),
    }
    for name, names in modules.items():
        module = ModuleType(name)
        for class_name in names:
            setattr(module, class_name, classes[class_name])
        monkeypatch.setitem(sys.modules, name, module)
    substitutions = ModuleType('launch.substitutions')
    substitutions.LaunchConfiguration = lambda key: SimpleNamespace(
        perform=lambda context: profile if key == 'max_linear_mps' else network_pause_resume)
    monkeypatch.setitem(sys.modules, 'launch.substitutions', substitutions)
    from nomad_navigation import robot_session_guard

    def token():
        if not allow_token:
            raise ValueError('Clock token missing or expired')
        return {'session_id': 'test-session'}

    monkeypatch.setattr(robot_session_guard, 'require_robot_session', token)
    path = Path(__file__).resolve().parents[1] / 'launch' / 'robot_bringup.launch.py'
    spec = importlib.util.spec_from_file_location('_test_bringup', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if stub_native:
        monkeypatch.setattr(module, '_native_executable',
                            lambda package, executable: f'/test/lib/{package}/{executable}')
    return module, classes


@pytest.mark.parametrize('profile,measured', [('0.05', '0.12'), ('0.1', '0.2'), ('0.2', '0.3')])
def test_only_zero_stage_starts_before_checker_success(monkeypatch, profile, measured):
    module, classes = load_launch(monkeypatch, profile)
    actions = module._configure(SimpleNamespace(is_shutdown=False))
    handler_type = classes['RegisterEventHandler']
    process_type = classes['ExecuteProcess']
    assert all(isinstance(action, handler_type) for action in actions[:6])
    processes = [action for action in actions if isinstance(action, process_type)]
    assert [action.name for action in processes] == [
        'nomad_controller', 'nomad_gateway', 'nomad_startup_check']
    assert f'max_linear_mps:={profile}' in processes[0].cmd
    assert f'max_measured_linear_mps:={measured}' in processes[0].cmd
    assert processes[1].cmd[-2:] == ['--max-linear-mps', profile]
    checker_handler = actions[5].args[0]
    after = checker_handler.on_exit(SimpleNamespace(returncode=0),
                                    SimpleNamespace(is_shutdown=False))
    hardware = [action for action in after if isinstance(action, process_type)]
    assert [action.name for action in hardware] == [
        'wheeltec_base', 'wheeltec_camera', 'base_link_transform']
    assert 'car_mode:=V550_akm' in hardware[0].cmd
    assert '0.01826' in hardware[2].cmd
    assert hardware[0].cmd[0] == '/test/lib/turn_on_wheeltec_robot/wheeltec_robot_node'
    assert hardware[2].cmd[0] == '/test/lib/tf2_ros/static_transform_publisher'
    assert all('run' not in action.cmd for action in (hardware[0], hardware[2]))
    for process in processes + hardware:
        assert 'arm' not in process.cmd and 'reset' not in process.cmd
        assert not getattr(process, 'respawn', False)


def test_checker_failure_never_starts_hardware(monkeypatch):
    module, classes = load_launch(monkeypatch)
    actions = module._configure(SimpleNamespace(is_shutdown=False))
    after = actions[5].args[0].on_exit(SimpleNamespace(returncode=1),
                                     SimpleNamespace(is_shutdown=False))
    assert len(after) == 1 and isinstance(after[0], classes['EmitEvent'])
    assert isinstance(after[0].event, classes['Shutdown'])


def test_required_process_exit_shuts_down_group(monkeypatch):
    module, classes = load_launch(monkeypatch)
    actions = module._configure(SimpleNamespace(is_shutdown=False))
    for registered in actions[:5]:
        emitted = registered.args[0].on_exit(SimpleNamespace(returncode=0),
                                            SimpleNamespace(is_shutdown=False))
        assert isinstance(emitted[0].event, classes['Shutdown'])
        assert registered.args[0].on_exit(SimpleNamespace(returncode=1),
                                         SimpleNamespace(is_shutdown=True)) == []
    assert actions[5].args[0].on_exit(SimpleNamespace(returncode=0),
                                     SimpleNamespace(is_shutdown=True)) == []


def test_missing_clock_token_prevents_any_process_construction(monkeypatch):
    module, _ = load_launch(monkeypatch, allow_token=False)
    with pytest.raises(ValueError, match='Clock token'):
        module._configure(SimpleNamespace(is_shutdown=False))


def test_unreviewed_speed_argument_rejected(monkeypatch):
    module, _ = load_launch(monkeypatch, profile='0.3')
    with pytest.raises(ValueError):
        module._configure(SimpleNamespace(is_shutdown=False))


@pytest.mark.parametrize('profile', ['0.05', '0.1', '0.2'])
def test_pause_resume_is_wired_to_both_api_and_controller(monkeypatch, profile):
    from nomad_navigation import robot_identity
    monkeypatch.setenv('NOMAD_TRANSPORT', 'api')
    monkeypatch.setattr(robot_identity, 'read_campus_client_config', lambda: dict(
        server_url='https://10.0.0.1:8443', ca_file='/test/ca', token_file='/test/token'))
    module, classes = load_launch(monkeypatch, profile=profile, network_pause_resume='true')
    actions = module._configure(SimpleNamespace(is_shutdown=False))
    processes = [row for row in actions if isinstance(row, classes['ExecuteProcess'])]
    assert 'network_pause_resume:=true' in processes[0].cmd
    assert f'max_linear_mps:={profile}' in processes[0].cmd
    assert '--max-linear-mps' in processes[1].cmd and profile == processes[1].cmd[-1]
    client = actions[6].args[0].on_exit(SimpleNamespace(returncode=0),
        SimpleNamespace(is_shutdown=False))[-1]
    assert '--network-pause-resume' in client.cmd and '--control-output' in client.cmd


@pytest.mark.parametrize('profile,value', [('0.05', 'true'), ('0.1', 'true'),
                                         ('0.2', 'true'), ('0.05', 'yes')])
def test_pause_resume_rejects_unsafe_launch_options(monkeypatch, profile, value):
    module, _ = load_launch(monkeypatch, profile=profile, network_pause_resume=value)
    with pytest.raises(ValueError):
        module._configure(SimpleNamespace(is_shutdown=False))


def test_missing_native_executable_prevents_any_process(monkeypatch):
    module, _ = load_launch(monkeypatch)
    created = []
    monkeypatch.setattr(module, 'ExecuteProcess', lambda **kwargs: created.append(kwargs))
    def unavailable(*_args):
        raise RuntimeError('Required robot executable is unavailable')
    monkeypatch.setattr(module, '_native_executable', unavailable)
    with pytest.raises(RuntimeError, match='executable is unavailable'):
        module._configure(SimpleNamespace(is_shutdown=False))
    assert created == []


@pytest.mark.parametrize('available,executable', [(False, False), (True, False), (True, True)])
def test_native_resolver_checks_libexec_without_running_it(monkeypatch, tmp_path, available, executable):
    module, _ = load_launch(monkeypatch, stub_native=False)
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_prefix = lambda package: str(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    path = tmp_path / 'lib' / 'tf2_ros' / 'static_transform_publisher'
    if available:
        path.parent.mkdir(parents=True)
        path.write_text('test-only; never executed')
    monkeypatch.setattr(module.os, 'access', lambda value, mode: executable and Path(value) == path)
    if available and executable:
        assert module._native_executable('tf2_ros', 'static_transform_publisher') == str(path)
    else:
        with pytest.raises(RuntimeError, match='executable is unavailable'):
            module._native_executable('tf2_ros', 'static_transform_publisher')


def test_default_profile_is_explicit_user_requested_point_two(monkeypatch):
    module, classes = load_launch(monkeypatch)
    description = module.generate_launch_description()
    argument = description.args[0][0]
    assert argument.default_value == '0.2'
    assert argument.choices == ['0.05', '0.1', '0.2']
    pause_argument = description.args[0][1]
    assert pause_argument.default_value == 'false'


def test_campus_client_starts_only_after_zero_check_and_is_required(monkeypatch):
    from nomad_navigation import robot_identity
    monkeypatch.setenv('NOMAD_TRANSPORT', 'api')
    monkeypatch.setattr(robot_identity, 'read_campus_client_config', lambda: {
        'server_url': 'https://10.201.91.120:8443',
        'ca_file': '/home/wheeltec/nomad_live_control/campus/server.crt',
        'token_file': '/home/wheeltec/nomad_live_control/campus/token'})
    module, classes = load_launch(monkeypatch, profile='0.05')
    actions = module._configure(SimpleNamespace(is_shutdown=False))
    initial = [action for action in actions if isinstance(action, classes['ExecuteProcess'])]
    assert [action.name for action in initial] == [
        'nomad_controller', 'nomad_gateway', 'nomad_startup_check']
    checker_handler = actions[6].args[0]
    after = checker_handler.on_exit(SimpleNamespace(returncode=0), SimpleNamespace(is_shutdown=False))
    api_client = [action for action in after if getattr(action, 'name', None) == 'nomad_campus_api_client'][0]
    assert api_client.cmd[:3] == ['python3', '-m', 'nomad_navigation.campus_api_client']
    assert '--control-output' in api_client.cmd
    assert '--token-file' in api_client.cmd
    assert api_client.cmd[api_client.cmd.index('--output-root') + 1] == '/home/wheeltec/nomad_live_control/runs/campus'
    assert not getattr(api_client, 'respawn', False)
    handler = actions[5].args[0]
    assert handler.target_action is api_client
    emitted = handler.on_exit(SimpleNamespace(returncode=1), SimpleNamespace(is_shutdown=False))
    assert isinstance(emitted[0].event, classes['Shutdown'])
    failed = checker_handler.on_exit(SimpleNamespace(returncode=1), SimpleNamespace(is_shutdown=False))
    assert not any(isinstance(action, classes['ExecuteProcess']) for action in failed)


def test_invalid_campus_config_prevents_process_construction(monkeypatch):
    from nomad_navigation import robot_identity
    monkeypatch.setenv('NOMAD_TRANSPORT', 'api')
    def bad_config():
        raise ValueError('unsafe credential')
    monkeypatch.setattr(robot_identity, 'read_campus_client_config', bad_config)
    module, _ = load_launch(monkeypatch)
    created = []
    monkeypatch.setattr(module, 'ExecuteProcess', lambda **kwargs: created.append(kwargs))
    with pytest.raises(ValueError, match='unsafe credential'):
        module._configure(SimpleNamespace(is_shutdown=False))
    assert created == []
