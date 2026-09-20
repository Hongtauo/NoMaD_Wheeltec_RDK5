"""Build the narrow Windows OpenSSH askpass adapter without reading credentials.

The executable reads the enrolled robot credential only when OpenSSH supplies its
exact password prompt. Passwords never enter this module or process arguments.
"""

import hashlib
import os
from pathlib import Path
import subprocess
import sys
import uuid


SOURCE = Path(__file__).resolve().parents[2] / 'docker' / 'ros2' / 'RobotSshAskpass.cs'
_BUILD_ERROR = 'Could not prepare the Windows robot SSH password helper.'


def _compiler_candidates():
    root = Path('C:/Windows/Microsoft.NET')
    return [root / architecture / 'v4.0.30319' / 'csc.exe'
            for architecture in ('Framework64', 'Framework')]


def ensure_askpass():
    """Return a source-versioned native helper, compiling locally when needed.

    No SSH connection, credential read, or robot action is performed here.
    """
    if sys.platform != 'win32':
        raise RuntimeError('Robot password autofill requires Windows OpenSSH.')
    local_data = os.environ.get('LOCALAPPDATA')
    if not local_data or not Path(local_data).is_absolute():
        raise RuntimeError(_BUILD_ERROR)
    temporary = None
    try:
        digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
        cache = Path(local_data) / 'SocialEmbodiedAgent' / 'ssh-askpass' / digest
        executable = cache / 'RobotSshAskpass.exe'
        if cache.is_symlink() or executable.is_symlink():
            raise RuntimeError(_BUILD_ERROR)
        if executable.is_file():
            return executable
        compiler = next((path for path in _compiler_candidates() if path.is_file()), None)
        if compiler is None:
            raise RuntimeError(_BUILD_ERROR)
        cache.mkdir(parents=True, exist_ok=True)
        temporary = cache / ('RobotSshAskpass.' + uuid.uuid4().hex + '.exe')
        result = subprocess.run(
            [str(compiler), '/nologo', '/optimize+', '/target:exe', '/platform:anycpu',
             '/out:' + str(temporary), str(SOURCE)],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=30, check=False,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0x08000000),
        )
        if result.returncode != 0 or not temporary.is_file():
            raise RuntimeError(_BUILD_ERROR)
        # Concurrent first runs use separate compiler outputs; each install is atomic.
        os.replace(temporary, executable)
        return executable
    except (OSError, subprocess.SubprocessError, ValueError):
        raise RuntimeError(_BUILD_ERROR) from None
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


def auto_ssh_options():
    """Return Popen keyword arguments enabling the helper for this SSH child only."""
    executable = ensure_askpass()
    environment = os.environ.copy()
    environment.update({
        'SSH_ASKPASS': str(executable),
        'SSH_ASKPASS_REQUIRE': 'force',
        'NOMAD_ROBOT_ASKPASS': '1',
    })
    environment.pop('SSH_ASKPASS_PROMPT', None)
    return {'env': environment}
