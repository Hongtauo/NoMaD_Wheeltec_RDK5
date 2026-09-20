"""Credential storage tests: never access the actual Windows credential store."""

import contextlib
import ctypes
import io
import unittest
from unittest import mock

import robot_credentials as credentials


class FakeApi:
    def __init__(self, password="test-only-password"):
        self.read_ok = True
        self.write_ok = True
        self.delete_ok = True
        self.freed = False
        self.calls = []
        encoded = password.encode("utf-16-le")
        self.buffer = ctypes.create_string_buffer(encoded, len(encoded))
        self.credential = credentials._CREDENTIALW()
        self.credential.Type = credentials._CRED_TYPE_GENERIC
        self.credential.TargetName = credentials.CREDENTIAL_TARGET
        self.credential.UserName = credentials.CREDENTIAL_USERNAME
        self.credential.CredentialBlobSize = len(encoded)
        self.credential.CredentialBlob = ctypes.cast(self.buffer, ctypes.POINTER(ctypes.c_ubyte))

    def CredReadW(self, target, kind, flags, output):
        self.calls.append(("read", target, kind, flags))
        if self.read_ok:
            ctypes.cast(output, ctypes.POINTER(ctypes.POINTER(credentials._CREDENTIALW)))[0] = ctypes.pointer(self.credential)
        return self.read_ok

    def CredWriteW(self, pointer, flags):
        value = ctypes.cast(pointer, ctypes.POINTER(credentials._CREDENTIALW)).contents
        self.calls.append(("write", value.TargetName, value.Type, flags))
        self.written_username = value.UserName
        self.written_persist = value.Persist
        self.written_blob = ctypes.string_at(value.CredentialBlob, value.CredentialBlobSize)
        return self.write_ok

    def CredDeleteW(self, target, kind, flags):
        self.calls.append(("delete", target, kind, flags))
        return self.delete_ok

    def CredFree(self, pointer):
        self.freed = True
        self.buffer_at_free = bytes(self.buffer)


class CredentialValidationTests(unittest.TestCase):
    def test_encoding_and_limit(self):
        self.assertEqual(credentials._validate_password("测试 x"), "测试 x".encode("utf-16-le"))
        self.assertEqual(len(credentials._validate_password("a" * 512)), 1024)
        self.assertEqual(len(credentials._validate_password("🔐" * 256)), 1024)

    def test_invalid_password_never_appears_in_error(self):
        for password in (None, 123, "", "x\ny", "x\ry", "x\0y", "x\u0085y", "x\u2028y", "x\u2029y", "x" * 513, "\ud800"):
            with self.subTest(kind=type(password).__name__):
                with self.assertRaises(credentials.CredentialError) as raised:
                    credentials._validate_password(password)
                if isinstance(password, str) and len(password) > 1:
                    self.assertNotIn(password, str(raised.exception))


class CredentialApiTests(unittest.TestCase):
    def test_read_exact_target_and_scrubs_before_free(self):
        api = FakeApi("test-测试")
        self.assertEqual(credentials._read_credential(api), "test-测试")
        self.assertEqual(api.calls, [("read", credentials.CREDENTIAL_TARGET, 1, 0)])
        self.assertTrue(api.freed)
        self.assertEqual(api.buffer_at_free, b"\0" * len(api.buffer))

    def test_missing_read_returns_none_without_free(self):
        api = FakeApi()
        api.read_ok = False
        with mock.patch.object(credentials, "_last_error", return_value=1168):
            self.assertIsNone(credentials._read_credential(api))
        self.assertFalse(api.freed)

    def test_read_failure_is_not_treated_as_missing(self):
        api = FakeApi()
        api.read_ok = False
        with mock.patch.object(credentials, "_last_error", return_value=5):
            with self.assertRaises(credentials.CredentialError):
                credentials._read_credential(api)

    def test_identity_mismatches_are_rejected_and_freed(self):
        for field, value in (("UserName", "other"), ("TargetName", "other"), ("Type", 2)):
            api = FakeApi()
            setattr(api.credential, field, value)
            with self.subTest(field=field):
                with self.assertRaises(credentials.CredentialError):
                    credentials._read_credential(api)
                self.assertTrue(api.freed)
                self.assertEqual(api.buffer_at_free, b"\0" * len(api.buffer))

    def test_bad_blob_size_is_rejected(self):
        for size in (0, 3, 1026):
            api = FakeApi()
            api.credential.CredentialBlobSize = size
            # Avoid a fixture smaller than the returned size during best-effort
            # cleanup of this intentionally malformed credential.
            api.buffer = ctypes.create_string_buffer(max(size, 1))
            api.credential.CredentialBlob = ctypes.cast(api.buffer, ctypes.POINTER(ctypes.c_ubyte))
            with self.subTest(size=size):
                with self.assertRaises(credentials.CredentialError):
                    credentials._read_credential(api)
                self.assertTrue(api.freed)

    def test_bad_stored_content_is_rejected_and_freed(self):
        api = FakeApi("secret\nsecret")
        with self.assertRaises(credentials.CredentialError):
            credentials._read_credential(api)
        self.assertTrue(api.freed)

    def test_write_identity_encoding_and_buffer_wipe(self):
        api = FakeApi()
        with mock.patch.object(credentials, "_wipe", wraps=credentials._wipe) as wipe:
            credentials._write_credential(api, "testing-测试")
        self.assertEqual(api.written_blob, "testing-测试".encode("utf-16-le"))
        self.assertEqual(api.written_username, "wheeltec")
        self.assertEqual(api.written_persist, 2)
        self.assertEqual(api.calls, [("write", credentials.CREDENTIAL_TARGET, 1, 0)])
        self.assertEqual(bytes(wipe.call_args.args[0]), b"\0" * len(api.written_blob))

    def test_failed_write_also_wipes_buffer(self):
        api = FakeApi()
        api.write_ok = False
        with mock.patch.object(credentials, "_wipe", wraps=credentials._wipe) as wipe:
            with self.assertRaises(credentials.CredentialError):
                credentials._write_credential(api, "test-only")
        self.assertEqual(bytes(wipe.call_args.args[0]), b"\0" * len("test-only".encode("utf-16-le")))

    def test_delete_fixed_target_only(self):
        api = FakeApi()
        self.assertTrue(credentials._delete_credential(api))
        self.assertEqual(api.calls, [("delete", credentials.CREDENTIAL_TARGET, 1, 0)])
        api.delete_ok = False
        with mock.patch.object(credentials, "_last_error", return_value=1168):
            self.assertFalse(credentials._delete_credential(api))
        with mock.patch.object(credentials, "_last_error", return_value=5):
            with self.assertRaises(credentials.CredentialError):
                credentials._delete_credential(api)

    def test_non_windows_optional_read_and_explicit_mutation_failure(self):
        with mock.patch.object(credentials.os, "name", "posix"), mock.patch.object(credentials, "_load_windows_api") as load:
            self.assertIsNone(credentials.read_password())
            with self.assertRaises(credentials.CredentialError):
                credentials.save_password("test-only")
            with self.assertRaises(credentials.CredentialError):
                credentials.delete_password()
        load.assert_not_called()

    def test_windows_public_api_uses_only_injected_fake(self):
        api = FakeApi("fake-only")
        with mock.patch.object(credentials.os, "name", "nt"), mock.patch.object(credentials, "_load_windows_api", return_value=api):
            self.assertEqual(credentials.read_password(), "fake-only")
            credentials.save_password("updated-fake")
            self.assertTrue(credentials.delete_password())


class CredentialCliTests(unittest.TestCase):
    def test_status_does_not_print_secret(self):
        stream = io.StringIO()
        with mock.patch.object(credentials, "read_password", return_value="never-display-me"), contextlib.redirect_stdout(stream):
            self.assertEqual(credentials.main(["--status"]), 0)
        self.assertEqual(stream.getvalue(), "Robot password is saved.\n")

    def test_save_prompts_without_echo(self):
        stream = io.StringIO()
        with mock.patch.object(credentials.getpass, "getpass", return_value="test-secret"), mock.patch.object(credentials, "save_password") as save, contextlib.redirect_stdout(stream):
            self.assertEqual(credentials.main(["--save"]), 0)
        save.assert_called_once_with("test-secret")
        self.assertNotIn("test-secret", stream.getvalue())

    def test_stdin_save_preserves_password_spaces(self):
        stream = io.StringIO()
        with mock.patch.object(credentials.sys, "stdin", io.StringIO(" test secret \r\n")), mock.patch.object(credentials, "save_password") as save, contextlib.redirect_stdout(stream):
            self.assertEqual(credentials.main(["--save-stdin"]), 0)
        save.assert_called_once_with(" test secret ")
        self.assertNotIn("test secret", stream.getvalue())

    def test_delete_status_only(self):
        stream = io.StringIO()
        with mock.patch.object(credentials, "delete_password", return_value=True), contextlib.redirect_stdout(stream):
            self.assertEqual(credentials.main(["--delete"]), 0)
        self.assertEqual(stream.getvalue(), "Saved robot password removed.\n")

    def test_cli_does_not_print_exception_contents(self):
        stream = io.StringIO()
        with mock.patch.object(credentials, "read_password", side_effect=OSError("secret text")), contextlib.redirect_stderr(stream):
            self.assertEqual(credentials.main(["--status"]), 1)
        self.assertNotIn("secret text", stream.getvalue())


if __name__ == "__main__":
    unittest.main()
