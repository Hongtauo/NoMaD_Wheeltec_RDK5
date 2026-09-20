#!/usr/bin/env python3
"""Create local TLS credentials once; never print keys or bearer tokens."""
import argparse
from datetime import datetime, timedelta, timezone
import ipaddress
import json
import os
from pathlib import Path
import secrets


def provision(directory, address):
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    address = str(ipaddress.IPv4Address(address))
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    files = [directory / name for name in ('server.key', 'server.crt', 'token', 'metadata.json')]
    if any(p.exists() for p in files):
        if not all(p.is_file() for p in files):
            raise RuntimeError('Partial credentials exist; refusing to overwrite them')
        metadata = json.loads(files[-1].read_text())
        if metadata.get('server_ip') != address:
            raise RuntimeError('Server IP changed: explicitly provision a NEW credentials directory')
        certificate = x509.load_pem_x509_certificate(files[1].read_bytes())
        if datetime.now(timezone.utc).replace(tzinfo=None) >= certificate.not_valid_after:
            raise RuntimeError('Certificate expired; explicit reprovisioning is required')
        print(json.dumps({'credentials': 'existing', 'server_ip': address}))
        return
    key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
    now = datetime.now(timezone.utc)
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'NoMaD campus inference')])
    certificate = (x509.CertificateBuilder().subject_name(subject).issuer_name(subject)
        .public_key(key.public_key()).serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(days=1)).not_valid_after(now + timedelta(days=90))
        .add_extension(x509.SubjectAlternativeName([
            x509.IPAddress(ipaddress.ip_address(address)),
            x509.IPAddress(ipaddress.ip_address('127.0.0.1'))]), critical=False)
        .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
        .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
        .sign(key, hashes.SHA256()))
    payloads = [
        key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                          serialization.NoEncryption()),
        certificate.public_bytes(serialization.Encoding.PEM),
        (secrets.token_urlsafe(48) + '\n').encode(),
        (json.dumps({'schema': 1, 'server_ip': address, 'created_utc': now.isoformat(),
                     'expires_utc': (now + timedelta(days=90)).isoformat()}, indent=2) + '\n').encode(),
    ]
    for path, data in zip(files, payloads):
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, 'wb') as output:
            output.write(data)
    print(json.dumps({'credentials': 'created', 'server_ip': address,
                      'certificate_sha256': certificate.fingerprint(hashes.SHA256()).hex()}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--credentials-dir', type=Path, required=True)
    parser.add_argument('--server-ip', required=True)
    args = parser.parse_args()
    provision(args.credentials_dir, args.server_ip)
