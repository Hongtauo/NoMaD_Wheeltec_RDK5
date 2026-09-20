"""No-hardware tests for provisioning and local-only visualization startup."""
import json
import os
from pathlib import Path
import ssl
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from provision_campus_api import provision
from campus_health import health


def test_provision_is_idempotent_and_refuses_address_change(tmp_path):
    pytest.importorskip('cryptography')
    provision(tmp_path, '10.201.91.120')
    before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    provision(tmp_path, '10.201.91.120')
    assert before == {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    assert len(before['token'].strip()) == 64
    with pytest.raises(RuntimeError, match='IP changed'):
        provision(tmp_path, '10.201.91.121')
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(str(tmp_path / 'server.crt'), str(tmp_path / 'server.key'))
    from cryptography import x509
    import ipaddress
    certificate = x509.load_pem_x509_certificate(before['server.crt'])
    san = certificate.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    assert ipaddress.ip_address('10.201.91.120') in san.get_values_for_type(x509.IPAddress)


def test_partial_credentials_are_not_overwritten(tmp_path):
    pytest.importorskip('cryptography')
    (tmp_path / 'token').write_text('existing')
    with pytest.raises(RuntimeError, match='Partial credentials'):
        provision(tmp_path, '10.201.91.120')
    assert (tmp_path / 'token').read_text() == 'existing'


@pytest.mark.parametrize('url', ['http://127.0.0.1:8443', 'https://user@127.0.0.1:8443',
                                 'https://127.0.0.1:8443/?secret=x'])
def test_health_rejects_unsafe_origins_before_network(url):
    with pytest.raises(ValueError):
        health(url, '/nonexistent-certificate')


def test_preview_is_local_only_and_closes(monkeypatch):
    pytest.importorskip('rclpy')
    from campus_rviz_bridge import CampusPreview
    monkeypatch.setenv('ROS_DOMAIN_ID', '87')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    preview = CampusPreview()
    try:
        assert preview.error is None
        assert preview._thread.is_alive()
    finally:
        preview.close()
    assert not preview._thread.is_alive()


def test_preview_rejects_robot_domain(monkeypatch):
    from campus_rviz_bridge import CampusPreview
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    with pytest.raises(RuntimeError, match='domain 87'):
        CampusPreview()
