#!/usr/bin/env python3
"""Compare sequential NoMaD calls on one thread versus fresh HTTP-like threads.

Diagnostic only: creates no ROS nodes, network endpoint, or robot commands.
Run only when the service is idle, as this loads another model on the GPU.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import threading
import time
import uuid

import numpy as np

from nomad_navigation.campus_api_server import load_backend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--iterations', type=int, default=8, choices=range(2, 21))
    parser.add_argument('--profile', action='store_true')
    args = parser.parse_args()
    root = Path(args.project_root).resolve()
    output = root / 'outputs/campus_api' / (
        'benchmark__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        + '__' + uuid.uuid4().hex[:8])
    output.mkdir(exist_ok=False)
    backend = load_backend(root, 'cuda')
    frames = [np.zeros((480, 640, 3), dtype=np.uint8) for _ in range(4)]
    measurements = {}
    sequence = 0

    def run_once(seq):
        started = time.perf_counter()
        _, model_ms = backend.infer(frames, seq)
        return {'inference_ms': model_ms, 'call_ms': 1000 * (time.perf_counter()-started)}

    for mode in ('main_thread', 'fresh_thread', 'persistent_worker'):
        samples = []
        with ThreadPoolExecutor(max_workers=1) as pool:
            for _ in range(args.iterations):
                sequence += 1
                if mode == 'main_thread':
                    samples.append(run_once(sequence))
                elif mode == 'persistent_worker':
                    samples.append(pool.submit(run_once, sequence).result())
                else:
                    values, failures = [], []
                    def once():
                        try:
                            values.append(run_once(sequence))
                        except BaseException as error:
                            failures.append(error)
                    worker = threading.Thread(target=once)
                    worker.start()
                    worker.join()
                    if failures:
                        raise failures[0]
                    samples.append(values[0])
        durations = [row['inference_ms'] for row in samples]
        measurements[mode] = dict(samples=samples, mean_ms=statistics.mean(durations),
                                  min_ms=min(durations), max_ms=max(durations))
    report = {'runtime': backend.runtime_info, 'warmup_ms': backend.warmup_ms,
              'iterations': args.iterations, 'input': 'four black RGB 640x480 images',
              'measurements': measurements, 'no_robot_or_network_access': True}
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    if args.profile:
        from torch.profiler import profile, ProfilerActivity
        with profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
                     record_shapes=True) as prof:
            run_once(sequence + 1)
        prof.export_chrome_trace(str(output / 'trace.json'))
        table = prof.key_averages().table(sort_by='self_cpu_time_total', row_limit=30)
        (output / 'profile_cpu.txt').write_text(table)
        print(table)
    print(json.dumps({'report': str(output / 'report.json'), 'runtime': report['runtime'],
                      'mean_ms': {key: value['mean_ms'] for key, value in measurements.items()}}))


if __name__ == '__main__':
    main()
