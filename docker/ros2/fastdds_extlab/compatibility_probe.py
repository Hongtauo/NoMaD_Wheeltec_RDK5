"""Local-only topic/service compatibility probe; no robot motion interfaces."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--role', choices=('peer', 'client'), required=True)
    p.add_argument('--seconds', type=float, default=20)
    args = p.parse_args()
    rclpy.init()
    n = rclpy.create_node('extlab_' + args.role)
    got = []
    if args.role == 'peer':
        pub = n.create_publisher(String, '/extlab/compatibility_text', 10)
        def publish():
            m = String(); m.data = 'humble-compatibility-check'; pub.publish(m)
        timer = n.create_timer(.1, publish)
        def reply(request, response):
            response.success = True; response.message = 'compatibility-ok'
            return response
        service = n.create_service(Trigger, '/extlab/compatibility_service', reply)
        future = None
    else:
        sub = n.create_subscription(String, '/extlab/compatibility_text', lambda m: got.append(m.data), 10)
        client = n.create_client(Trigger, '/extlab/compatibility_service')
        future = None
    libraries = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if any(s in line for s in ('libfastrtps.so', 'librmw_fastrtps', 'libfastcdr.so'))})
    print(json.dumps({'role': args.role, 'loaded_libraries': libraries}), flush=True)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
        if args.role == 'client' and future is None and client.service_is_ready():
            future = client.call_async(Trigger.Request())
    ok = bool(future and future.done() and future.exception() is None and future.result().success)
    result = {'role': args.role, 'received_messages': len(got),
              'valid_messages': sum(x == 'humble-compatibility-check' for x in got), 'service_ok': ok}
    print(json.dumps(result), flush=True)
    n.destroy_node(); rclpy.shutdown()
    if args.role == 'client' and (not ok or len(got) < 5):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
