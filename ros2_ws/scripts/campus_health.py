#!/usr/bin/env python3
"""Read-only authenticated-server TLS health probe; no proxy, ROS, or motion."""
import argparse
import http.client
import ipaddress
import json
import ssl
from urllib.parse import urlsplit


def health(url, ca_file):
    parts = urlsplit(url)
    if (parts.scheme != 'https' or parts.username or parts.password or parts.path not in ('', '/')
            or parts.query or parts.fragment):
        raise ValueError('Expected https://numeric-server-ip:port')
    ipaddress.ip_address(parts.hostname)
    context = ssl.create_default_context(cafile=str(ca_file))
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    connection = http.client.HTTPSConnection(parts.hostname, parts.port or 443,
                                            context=context, timeout=5)
    try:
        connection.request('GET', '/healthz')
        response = connection.getresponse()
        payload = response.read(16385)
        if len(payload) > 16384:
            raise ValueError('Oversized health response')
        result = json.loads(payload)
        print(json.dumps({'http_status': response.status, 'server': result}))
        if response.status != 200:
            raise RuntimeError('Server is not ready')
        return result
    finally:
        connection.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server-url', required=True)
    parser.add_argument('--ca-file', required=True)
    args = parser.parse_args()
    health(args.server_url, args.ca_file)
