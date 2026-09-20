#!/usr/bin/env python3
"""Plot archived camera observations and model-local paths; never publish ROS."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    candidates = list((run / 'model').glob('*/manifest.json'))
    if len(candidates) != 1:
        raise ValueError('Expected one model manifest in the specified run')
    model_dir = candidates[0].parent
    manifest = json.loads(candidates[0].read_text())
    rows = [json.loads(line) for line in (model_dir / 'inference.jsonl').read_text().splitlines() if line]
    if not rows:
        raise ValueError('No archived inference rows to plot')
    selected = np.unique(np.linspace(0, len(rows) - 1, 3, dtype=int))
    figure, axes = plt.subplots(len(selected), 2, figsize=(11, 3.7 * len(selected)), squeeze=False)
    for position, index in enumerate(selected):
        row = rows[index]
        archive = (model_dir / row['archive']).resolve()
        if model_dir.resolve() not in archive.parents:
            raise ValueError('Archive path must stay inside the selected model run')
        with np.load(archive, allow_pickle=False) as data:
            axes[position, 0].imshow(data['current_rgb'])
            stamp = datetime.fromtimestamp(row['observation_stamp_ns'] / 1e9, timezone.utc)
            axes[position, 0].set_title(f'Live camera / output {index} / {stamp:%H:%M:%S} UTC')
            axes[position, 0].axis('off')
            axis = axes[position, 1]
            for action in data['trajectories'] * manifest['waypoint_scale']:
                axis.plot(np.r_[0, action[:, 1]], np.r_[0, action[:, 0]], '.-', alpha=.6)
            x, y = data['waypoint']
            axis.scatter([y], [x], c='red', marker='x', s=90, label='Preview waypoint')
            axis.set(xlabel='lateral y (nominal m)', ylabel='forward x (nominal m)')
            axis.set_aspect('equal', adjustable='datalim')
            axis.invert_xaxis()  # +y is robot left; forward x is up.
            axis.grid(True)
            axis.legend()
    figure.suptitle('Live-camera NoMaD inference only — no robot motion / no collision guarantee')
    figure.tight_layout(rect=(0, 0, 1, .97))
    destination = run / ('preview__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '.png')
    figure.savefig(destination, dpi=130)
    plt.close(figure)
    print(destination)


if __name__ == '__main__':
    main()
