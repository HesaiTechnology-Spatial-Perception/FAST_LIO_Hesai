#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Reject stale launch files and accidental private configuration in source/install catalogs."""
import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET

MODELS = ('jt16', 'jt32', 'jt64p', 'jt128', 'mt60')


def check(root, ros):
    configs = {'MT60.yaml' if m == 'mt60' else m + '.yaml' for m in MODELS}
    assert {p.name for p in (root / 'config').iterdir()} == configs, 'Unexpected/missing model configuration'
    suffix = '.launch' if ros == 1 else '.launch.py'
    launches = {'mapping_' + m + suffix for m in MODELS}
    if ros == 2:
        launches.add('mapping.launch.py')
    actual = {p.name for p in (root / 'launch').iterdir() if p.name != '__pycache__'}
    assert actual == launches, ('Unexpected/missing launch entry', actual ^ launches)
    for name in launches:
        path = root / 'launch' / name
        if ros == 1:
            ET.parse(path)
        else:
            compile(path.read_text(), str(path), 'exec')
        refs = set(re.findall(r'[A-Za-z0-9_]+\.yaml', path.read_text()))
        assert refs and refs <= configs, (name, 'Missing model config', refs - configs)
    print('Public source/install catalog passed:', root, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--ros', type=int, choices=(1, 2), required=True)
    args = parser.parse_args()
    check(args.root, args.ros)
