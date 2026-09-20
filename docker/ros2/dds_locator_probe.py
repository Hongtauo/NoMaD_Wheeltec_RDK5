"""Bounded, receive-only ROS probe with IPv4 RTPS discovery header inspection.

Run in a temporary Linux container. Requires CAP_NET_RAW for AF_PACKET.
Does not publish commands or persist camera payloads. Output is JSON metadata.
"""
import argparse
import collections
import json
import socket
import struct
import threading
import time


LOCATORS = {0x002f: 'unicast', 0x0030: 'multicast',
            0x0031: 'default_unicast', 0x0032: 'metatraffic_unicast',
            0x0033: 'metatraffic_multicast', 0x0048: 'default_multicast'}


def decode_rtps(data):
    """Decode SPDP parameter lists only; deliberately omit topic data."""
    if len(data) < 20 or data[:4] != b'RTPS':
        return None
    result = {'guid_prefix': data[8:20].hex(), 'submessages': [], 'spdp': []}
    offset = 20
    while offset + 4 <= len(data):
        kind, flags = data[offset:offset + 2]
        endian = '<' if flags & 1 else '>'
        size = struct.unpack_from(endian + 'H', data, offset + 2)[0]
        stop = offset + 4 + size if size else len(data)
        if stop > len(data):
            break
        result['submessages'].append(kind)
        body = data[offset + 4:stop]
        # SPDP builtin participant writer; DATA, no inline QoS, payload present.
        if kind == 0x15 and flags & 4 and not flags & 2 and len(body) >= 24 and body[8:12] == bytes.fromhex('000100c2'):
            payload_at = 4 + struct.unpack_from(endian + 'H', body, 2)[0]
            payload = body[payload_at:]
            if len(payload) >= 4 and payload[:2] in (b'\x00\x02', b'\x00\x03'):
                pe = '<' if payload[1] == 3 else '>'
                pos = 4
                while pos + 4 <= len(payload):
                    pid, plen = struct.unpack_from(pe + 'HH', payload, pos)
                    pos += 4
                    if pid == 1 or pos + plen > len(payload):
                        break
                    value = payload[pos:pos + plen]
                    if pid in LOCATORS and plen == 24:
                        lk, port = struct.unpack_from(pe + 'iI', value)
                        address = socket.inet_ntop(socket.AF_INET, value[20:24]) if lk == 1 else value[8:24].hex()
                        result['spdp'].append({'type': LOCATORS[pid], 'kind': lk, 'address': address, 'port': port})
                    pos += (plen + 3) & ~3
        offset = stop
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--robot', default='192.168.0.100')
    parser.add_argument('--capture-only', action='store_true')
    args = parser.parse_args()
    # ETH_P_ALL includes outgoing packets; filter to IPv4/RTPS below.
    raw = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
    raw.settimeout(.2)
    stopped = threading.Event()
    flows = collections.Counter()
    announcements = {}

    def capture():
        while not stopped.is_set():
            try:
                packet, info = raw.recvfrom(65535)
            except socket.timeout:
                continue
            if len(packet) < 42 or packet[12:14] != b'\x08\x00':
                continue
            ip = packet[14:]
            ihl = (ip[0] & 15) * 4
            if ip[9] != 17 or len(ip) < ihl + 8:
                continue
            src, dst = socket.inet_ntoa(ip[12:16]), socket.inet_ntoa(ip[16:20])
            if args.robot not in (src, dst) and dst != '239.255.0.1':
                continue
            sport, dport, length = struct.unpack_from('!HHH', ip, ihl)
            decoded = decode_rtps(ip[ihl + 8:ihl + length])
            if decoded is None:
                continue
            flow = (info[0], src, sport, dst, dport, decoded['guid_prefix'])
            flows[flow] += 1
            if decoded['spdp']:
                announcements[decoded['guid_prefix']] = decoded['spdp']

    worker = threading.Thread(target=capture, daemon=True)
    worker.start()
    print(json.dumps({'status': 'capture_ready', 'seconds': args.seconds}), flush=True)
    frames = []
    node = None
    if not args.capture_only:
        import rclpy
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init()
        node = rclpy.create_node('dds_locator_receive_probe')
        sub = node.create_subscription(CompressedImage, '/camera/color/image_raw/compressed',
                                       lambda msg: frames.append(time.monotonic()), qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        if node is not None:
            rclpy.spin_once(node, timeout_sec=.1)
        else:
            stopped.wait(.1)
    summary = {'frames': len(frames), 'announcements': announcements}
    if node is not None:
        summary['camera_publishers'] = node.count_publishers('/camera/color/image_raw/compressed')
        summary['nodes'] = node.get_node_names_and_namespaces()
        node.destroy_node()
        rclpy.shutdown()
    stopped.set()
    worker.join(timeout=1)
    raw.close()
    summary['flows'] = [dict(zip(('interface', 'src', 'sport', 'dst', 'dport', 'guid_prefix'), f), packets=n)
                        for f, n in flows.items()]
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
