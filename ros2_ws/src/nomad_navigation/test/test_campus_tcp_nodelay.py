"""TCP_NODELAY parity without changing the existing HTTPS safety budget."""
import errno
import socket
import ssl
import time
import unittest
from unittest.mock import Mock, call, patch

from nomad_navigation.campus_client_core import (
    DeadlineHTTPSConnection, REQUEST_DEADLINE_SEC,
)


class TCPNoDelayTests(unittest.TestCase):
    def connection(self, host='127.0.0.1'):
        raw, wrapped, context = Mock(), Mock(), Mock()
        context.wrap_socket.return_value = wrapped
        connection = DeadlineHTTPSConnection(host, 8443, context, 10.8)
        return connection, raw, wrapped, context

    def test_option_set_after_tcp_connect_before_tls_for_both_families(self):
        for host, family in (('127.0.0.1', socket.AF_INET), ('::1', socket.AF_INET6)):
            with self.subTest(host=host):
                connection, raw, wrapped, context = self.connection(host)
                operations = Mock()
                operations.attach_mock(raw, 'raw')
                operations.attach_mock(context, 'context')
                with patch('nomad_navigation.campus_client_core.socket.socket', return_value=raw) as factory, \
                        patch('nomad_navigation.campus_client_core.time.monotonic',
                              side_effect=[10., 10.1, 10.2, 10.3]):
                    connection.connect()
                factory.assert_called_once_with(family, socket.SOCK_STREAM)
                raw.setsockopt.assert_called_once_with(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                context.wrap_socket.assert_called_once_with(
                    raw, server_hostname=host, do_handshake_on_connect=False)
                order = operations.mock_calls
                self.assertLess(order.index(call.raw.connect((host, 8443))),
                                order.index(call.raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)))
                self.assertLess(order.index(call.raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)),
                                order.index(call.context.wrap_socket(
                                    raw, server_hostname=host, do_handshake_on_connect=False)))
                self.assertAlmostEqual(raw.settimeout.call_args_list[0].args[0], .8)
                self.assertAlmostEqual(raw.settimeout.call_args_list[1].args[0], .7)
                self.assertAlmostEqual(wrapped.settimeout.call_args_list[0].args[0], .6)
                self.assertAlmostEqual(wrapped.settimeout.call_args_list[1].args[0], .5)
                wrapped.do_handshake.assert_called_once_with()
                self.assertEqual(connection.deadline, 10.8)
                self.assertIs(connection._active_socket, wrapped)
                connection.cancel()
                wrapped.shutdown.assert_called_once_with(socket.SHUT_RDWR)
                wrapped.close.assert_called_once_with()

    def test_only_unsupported_option_has_stdlib_compatible_fallback(self):
        connection, raw, wrapped, context = self.connection()
        raw.setsockopt.side_effect = OSError(errno.ENOPROTOOPT, 'option unsupported')
        with patch('nomad_navigation.campus_client_core.socket.socket', return_value=raw), \
                patch('nomad_navigation.campus_client_core.time.monotonic', return_value=10.):
            connection.connect()
        wrapped.do_handshake.assert_called_once_with()
        self.assertEqual(connection.deadline, 10.8)

    def test_other_option_errors_propagate_and_raw_socket_stays_cancellable(self):
        connection, raw, wrapped, context = self.connection()
        expected = OSError(errno.EPERM, 'option denied')
        raw.setsockopt.side_effect = expected
        with patch('nomad_navigation.campus_client_core.socket.socket', return_value=raw), \
                patch('nomad_navigation.campus_client_core.time.monotonic', return_value=10.):
            with self.assertRaises(OSError) as raised:
                connection.connect()
        self.assertIs(raised.exception, expected)
        context.wrap_socket.assert_not_called()
        self.assertIs(connection._active_socket, raw)
        connection.cancel()
        raw.shutdown.assert_called_once_with(socket.SHUT_RDWR)
        raw.close.assert_called_once_with()

    def test_option_does_not_refresh_expiring_deadline(self):
        connection, raw, wrapped, context = self.connection()
        with patch('nomad_navigation.campus_client_core.socket.socket', return_value=raw), \
                patch('nomad_navigation.campus_client_core.time.monotonic', side_effect=[10., 10.8]):
            with self.assertRaises(TimeoutError):
                connection.connect()
        raw.setsockopt.assert_called_once_with(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        context.wrap_socket.assert_not_called()
        self.assertEqual(connection.deadline, 10.8)
        self.assertEqual(REQUEST_DEADLINE_SEC, .8)
        connection.cancel()

    def test_cancellation_during_option_cannot_continue_to_tls(self):
        connection, raw, wrapped, context = self.connection()
        raw.setsockopt.side_effect = lambda *args: connection.cancel()
        with patch('nomad_navigation.campus_client_core.socket.socket', return_value=raw), \
                patch('nomad_navigation.campus_client_core.time.monotonic', return_value=10.):
            with self.assertRaises(TimeoutError):
                connection.connect()
        context.wrap_socket.assert_not_called()
        raw.shutdown.assert_called_once_with(socket.SHUT_RDWR)
        raw.close.assert_called_once_with()

    def test_already_expired_request_does_not_connect_or_set_option(self):
        connection, raw, wrapped, context = self.connection()
        with patch('nomad_navigation.campus_client_core.socket.socket', return_value=raw), \
                patch('nomad_navigation.campus_client_core.time.monotonic', return_value=10.8):
            with self.assertRaises(TimeoutError):
                connection.connect()
        raw.connect.assert_not_called()
        raw.setsockopt.assert_not_called()
        context.wrap_socket.assert_not_called()
        connection.cancel()

    def test_real_socket_option_is_inherited_by_ssl_wrapper(self):
        # Loopback only. Skip the handshake here to isolate the OS socket
        # option; test_campus_client_tls separately exercises real TLS/CA.
        context = ssl.create_default_context()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen(1)
            connection = DeadlineHTTPSConnection(
                '127.0.0.1', listener.getsockname()[1], context,
                time.monotonic() + REQUEST_DEADLINE_SEC)
            try:
                with patch.object(ssl.SSLSocket, 'do_handshake') as handshake:
                    connection.connect()
                handshake.assert_called_once_with()
                self.assertIsInstance(connection.sock, ssl.SSLSocket)
                self.assertEqual(connection.sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), 1)
                self.assertTrue(context.check_hostname)
                self.assertEqual(context.verify_mode, ssl.CERT_REQUIRED)
            finally:
                connection.cancel()
                connection.close()


if __name__ == '__main__':
    unittest.main()
