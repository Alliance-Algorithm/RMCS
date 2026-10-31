#!/usr/bin/env python3
"""Compose a new, disabled recovery configuration without launching a vehicle."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    fragment=ROOT/'rmcs_ws/src/rmcs_bringup/config/wheel-leg-self-righting-v202.yaml'
    base=yaml.safe_load(args.base.read_text())
    extra=yaml.safe_load(fragment.read_text())
    merged=copy.deepcopy(base)
    components=merged['rmcs_executor']['ros__parameters']['components']
    required=('WheelLegStateEstimator','WheelLegTrajectoryController','WheelLegDriveBridge')
    for name in required:
        if not any(name in c for c in components):raise ValueError('Missing traditional component: '+name)
    if 'wheelleg_self_righting'in merged or any('WheelLegSelfRightingController'in c for c in components):
        raise ValueError('Recovery is already present; do not duplicate component instances')
    components+=extra['rmcs_executor']['ros__parameters']['components']
    merged['wheelleg_self_righting']=extra['wheelleg_self_righting']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x')as file:yaml.safe_dump(merged,file,allow_unicode=True,sort_keys=False)
    print(json.dumps(dict(output=str(args.output.resolve()),components=len(components),enabled=False,
        base_sha256=hashlib.sha256(args.base.read_bytes()).hexdigest(),
        fragment_sha256=hashlib.sha256(fragment.read_bytes()).hexdigest(),
        note='The inherited plant and its coordinates are unchanged. v202 recovery stays disabled.')))


if __name__=='__main__':main()
