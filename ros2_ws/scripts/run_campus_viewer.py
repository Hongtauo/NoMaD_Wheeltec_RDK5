#!/usr/bin/env python3
"""Independent read-only snapshot + local-domain RViz; no API credentials."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import threading


def stop_viewer(process):
    if process is None or process.poll() is not None:
        return
    # RViz is a direct child, no shell wrapper or robot process group.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5.)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=3.)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--snapshot-file', default='/preview/latest.json')
    args = parser.parse_args(argv)
    if (os.environ.get('ROS_DOMAIN_ID') != '87'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise ValueError('Viewer requires isolated local-only ROS domain 87')
    if not Path(args.snapshot_file).is_absolute():
        raise ValueError('Snapshot file must be an absolute local path')
    stopping = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stopping.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    preview = rviz = None
    try:
        from campus_rviz_bridge import CampusPreview
        preview = CampusPreview(snapshot_path=args.snapshot_file)
        config = Path(__file__).resolve().parents[1] / 'src/nomad_navigation/rviz/nomad_campus.rviz'
        rviz = subprocess.Popen(['rviz2', '-d', str(config)])
        while not stopping.wait(.1):
            if preview.error:
                raise RuntimeError('Local preview failed: ' + preview.error)
            code = rviz.poll()
            if code is not None:
                return code
        return 0
    finally:
        try:
            stop_viewer(rviz)
        finally:
            if preview is not None:
                preview.close()
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
