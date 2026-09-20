#!/usr/bin/env python3
"""Headless TLS inference with a bounded, independent preview-file feed."""
import argparse
from pathlib import Path
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--preview-file', default=str(
        Path(__file__).resolve().parents[2] / 'outputs/campus_api/preview/latest.json'))
    parser.add_argument('--rviz', action='store_true')
    options, server_args = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    preview = None
    try:
        if options.rviz:
            print('--rviz is now independent: start docker/ros2/start_campus_rviz.ps1; API remains headless.', flush=True)
        try:
            from campus_preview_snapshot import SnapshotWriter
            preview = SnapshotWriter(options.preview_file)
        except Exception as error:
            print('Preview file unavailable (' + type(error).__name__ + '); inference remains independent.', flush=True)
        from nomad_navigation.campus_api_server import main as serve
        return serve(server_args, on_result=preview.submit if preview else None)
    finally:
        if preview:
            try:
                if not preview.close():
                    print('Preview file cleanup did not finish before its bound; expired files are never replayed.', flush=True)
            except Exception as error:
                print('Preview cleanup warning: ' + type(error).__name__, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
