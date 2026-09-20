"""Store this project's robot password in Windows Credential Manager.

Only one fixed, current-user credential is accessed. Passwords must never be
included in command arguments, source files, status messages, or logs. Python
strings cannot be reliably erased; native credential buffers are cleared on a
best-effort basis as soon as their contents have been copied or submitted.
"""

import argparse
import ctypes
import getpass
import os
import sys


CREDENTIAL_TARGET = "SocialEmbodiedAgent/robot/wheeltec@192.168.0.100"
CREDENTIAL_USERNAME = "wheeltec"
MAX_BLOB_BYTES = 1024
_CRED_TYPE_GENERIC = 1
_CRED_PERSIST_LOCAL_MACHINE = 2
_ERROR_NOT_FOUND = 1168


class CredentialError(RuntimeError):
    """A credential operation failed without exposing sensitive contents."""


class _FILETIME(ctypes.Structure):
    _fields_ = [("dwLowDateTime", ctypes.c_uint32),
                ("dwHighDateTime", ctypes.c_uint32)]


class _CREDENTIALW(ctypes.Structure):
    _fields_ = [
        ("Flags", ctypes.c_uint32),
        ("Type", ctypes.c_uint32),
        ("TargetName", ctypes.c_wchar_p),
        ("Comment", ctypes.c_wchar_p),
        ("LastWritten", _FILETIME),
        ("CredentialBlobSize", ctypes.c_uint32),
        ("CredentialBlob", ctypes.POINTER(ctypes.c_ubyte)),
        ("Persist", ctypes.c_uint32),
        ("AttributeCount", ctypes.c_uint32),
        ("Attributes", ctypes.c_void_p),
        ("TargetAlias", ctypes.c_wchar_p),
        ("UserName", ctypes.c_wchar_p),
    ]


def _load_windows_api():
    api = ctypes.WinDLL("advapi32", use_last_error=True)
    api.CredReadW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32,
                             ctypes.c_uint32,
                             ctypes.POINTER(ctypes.POINTER(_CREDENTIALW))]
    api.CredReadW.restype = ctypes.c_int
    api.CredWriteW.argtypes = [ctypes.POINTER(_CREDENTIALW), ctypes.c_uint32]
    api.CredWriteW.restype = ctypes.c_int
    api.CredDeleteW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32,
                               ctypes.c_uint32]
    api.CredDeleteW.restype = ctypes.c_int
    api.CredFree.argtypes = [ctypes.c_void_p]
    api.CredFree.restype = None
    return api


def _last_error():
    return ctypes.get_last_error()


def _validate_password(password):
    if not isinstance(password, str) or not password:
        raise CredentialError("Robot password must be a nonempty string.")
    if any(character in password for character in ("\r", "\n", "\0", "\u0085", "\u2028", "\u2029")):
        raise CredentialError("Robot password must be a single line without NUL characters.")
    try:
        encoded = password.encode("utf-16-le")
    except UnicodeError:
        raise CredentialError("Robot password contains invalid Unicode.") from None
    if len(encoded) > MAX_BLOB_BYTES:
        raise CredentialError("Robot password exceeds the credential storage limit.")
    return encoded


def _wipe(pointer, size):
    if pointer and size:
        try:
            ctypes.memset(pointer, 0, size)
        except (OSError, ValueError):
            # Clearing is best effort and must not mask the real operation.
            pass


def _read_credential(api):
    pointer = ctypes.POINTER(_CREDENTIALW)()
    if not api.CredReadW(CREDENTIAL_TARGET, _CRED_TYPE_GENERIC, 0,
                         ctypes.byref(pointer)):
        if _last_error() == _ERROR_NOT_FOUND:
            return None
        raise CredentialError("Unable to read the saved robot credential.")
    if not pointer:
        raise CredentialError("Windows returned an invalid robot credential.")
    credential = pointer.contents
    try:
        if (credential.Type != _CRED_TYPE_GENERIC
                or credential.TargetName != CREDENTIAL_TARGET
                or credential.UserName != CREDENTIAL_USERNAME):
            raise CredentialError("The saved robot credential has an unexpected identity.")
        size = credential.CredentialBlobSize
        if not credential.CredentialBlob or not 0 < size <= MAX_BLOB_BYTES or size % 2:
            raise CredentialError("The saved robot credential has an invalid password payload.")
        try:
            password = ctypes.string_at(credential.CredentialBlob, size).decode("utf-16-le")
        except UnicodeError:
            raise CredentialError("The saved robot credential has an invalid password payload.") from None
        _validate_password(password)
        return password
    finally:
        if 0 < credential.CredentialBlobSize <= MAX_BLOB_BYTES:
            _wipe(credential.CredentialBlob, credential.CredentialBlobSize)
        api.CredFree(pointer)


def _write_credential(api, password):
    encoded = _validate_password(password)
    buffer = ctypes.create_string_buffer(encoded, len(encoded))
    credential = _CREDENTIALW()
    credential.Type = _CRED_TYPE_GENERIC
    credential.TargetName = CREDENTIAL_TARGET
    credential.UserName = CREDENTIAL_USERNAME
    credential.CredentialBlobSize = len(encoded)
    credential.CredentialBlob = ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte))
    credential.Persist = _CRED_PERSIST_LOCAL_MACHINE
    try:
        if not api.CredWriteW(ctypes.byref(credential), 0):
            raise CredentialError("Unable to save the robot credential.")
    finally:
        _wipe(buffer, len(encoded))


def _delete_credential(api):
    if api.CredDeleteW(CREDENTIAL_TARGET, _CRED_TYPE_GENERIC, 0):
        return True
    if _last_error() == _ERROR_NOT_FOUND:
        return False
    raise CredentialError("Unable to delete the saved robot credential.")


def read_password():
    """Return this user's saved password, or None when absent/non-Windows."""
    if os.name != "nt":
        return None
    return _read_credential(_load_windows_api())


def save_password(password):
    """Save under the fixed target for this Windows user; never echo it."""
    if os.name != "nt":
        raise CredentialError("Saving a robot credential requires Windows.")
    _write_credential(_load_windows_api(), password)


def delete_password():
    """Delete only the fixed target; return whether an entry existed."""
    if os.name != "nt":
        raise CredentialError("Deleting a robot credential requires Windows.")
    return _delete_credential(_load_windows_api())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--save", action="store_true", help="Prompt once and save the robot password")
    mode.add_argument("--delete", action="store_true", help="Remove this project's saved robot password")
    mode.add_argument("--status", action="store_true", help="Show whether the robot password is saved")
    mode.add_argument("--save-stdin", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        if args.save or args.save_stdin:
            if args.save_stdin:
                password = sys.stdin.read(MAX_BLOB_BYTES + 2)
                if password.endswith("\r\n"):
                    password = password[:-2]
                elif password.endswith("\n"):
                    password = password[:-1]
            else:
                password = getpass.getpass("Robot password: ")
            save_password(password)
            print("Robot password saved in Windows Credential Manager.")
        elif args.delete:
            removed = delete_password()
            print("Saved robot password removed." if removed else "No saved robot password.")
        else:
            print("Robot password is saved." if read_password() is not None
                  else "No saved robot password.")
        return 0
    except (CredentialError, OSError):
        # Do not let Windows/third-party exception text reveal a secret.
        print("Robot credential operation failed; credential contents were not displayed.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
