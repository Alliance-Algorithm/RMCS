#!/usr/bin/env python3
"""Reproducible offline identification of the wheel/leg numeric sweep archive.

Raw files are read only. Integer host clocks are preserved; fits and controller
comparisons are candidates until independently validated on the real mechanism.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

S = "/wheel_leg/sweep/"
AXES = ("left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint",
        "left_wheel", "right_wheel")


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean_json(v) for v in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean_json(value), ensure_ascii=False, indent=2,
                               allow_nan=False) + "\n")


def array(data, name, default=0):
    return data[name].to_numpy() if name in data else np.full(len(data), default)


def load_source(path, cache):
    tag = hashlib.sha256(str(path.resolve()).encode()).hexdigest()[:12]
    binary, metadata = cache / (tag + ".pkl"), cache / (tag + ".json")
    stat = path.stat()
    if metadata.exists() and binary.exists():
        info = json.loads(metadata.read_text())
        if info["bytes"] == stat.st_size and info["mtime_ns"] == stat.st_mtime_ns:
            return pd.read_pickle(binary), info
    with path.open(encoding="utf-8-sig", newline="") as stream:
        header = next(csv.reader(stream))
    digest, malformed, lines = hashlib.sha256(), [], 0
    with path.open("rb") as stream:
        for lines, line in enumerate(stream, 1):
            digest.update(line)
            if line.count(b",") + 1 != len(header):
                malformed.append(lines)
    # A cut final writer row is recorded and excluded; never silently repair an
    # interior corruption or reinterpret an incomplete row as valid zeros.
    if malformed and malformed != [lines]:
        raise ValueError(f"Interior malformed rows in {path}: {malformed[:8]}")
    selected = [name for name in header if name.startswith(S)
                or name in ("index", "/wheel_leg/dr16_fresh", "/wheel_leg/enable_request")
                or name.startswith("/wheel_leg/") and any(name.endswith("/" + suffix)
                    for suffix in ("angle", "velocity", "torque", "max_torque", "control_torque",
                                   "feedback_sequence", "feedback_steady_ns", "tau_frame_api",
                                   "tx_kind", "tx_queued_steady_ns", "temperature_c"))]
    types = {name: "int64" for name in selected if name.endswith(("_ns", "feedback_sequence"))
             or name == "index"}
    data = pd.read_csv(path, usecols=selected, dtype=types,
                       nrows=lines - 1 - len(malformed))
    info = {"file": path.name, "path": str(path.resolve()), "bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns, "sha256": digest.hexdigest(),
            "columns": len(header), "records": len(data), "malformed_lines_excluded": malformed}
    data.to_pickle(binary)
    write_json(metadata, info)
    return data, info


def finite_stats(values):
    x = np.asarray(values)
    x = x[np.isfinite(x)]
    if not len(x):
        return {"samples": 0}
    return {"samples": len(x), "min": float(x.min()), "max": float(x.max()),
            "mean": float(x.mean()), "std": float(x.std()),
            "rms": float(np.sqrt(np.mean(x*x))),
            "p05_p50_p95": np.percentile(x, [5, 50, 95]).tolist()}


def audit_source(data, info):
    active = (array(data, S+"state") == 3) & (array(data, S+"reference_active") == 1)
    ns = array(data, S+"control_steady_ns")
    dt = np.diff(ns)*1e-9
    report = dict(info, duration_s=float((ns[-1]-ns[0])*1e-9), active_records=int(active.sum()),
        states=data[S+"state"].value_counts().to_dict(),
        experiment=np.unique(array(data, S+"experiment")).tolist(),
        active_modes=np.unique(array(data, S+"mode")[active]).tolist(),
        active_sides=np.unique(array(data, S+"side")[active]).tolist(),
        revisions=np.unique(array(data, S+"excitation_revision", -1)).tolist(),
        nonpositive_clock_steps=int(np.sum(dt <= 0)), gaps_over_3ms=int(np.sum(dt > .003)),
        dt_s=finite_stats(dt), validation_records=int(np.sum(active & (array(data,S+"validation")==1))),
        completed_records=int(np.sum(array(data,S+"completed")==1)), axes={}, stages=[])
    if not active.any():
        return report
    for axis in AXES:
        m, p = "/wheel_leg/"+axis+"/", S+axis+"/"
        if m+"velocity" not in data:
            continue
        feedback_ns=array(data,m+"feedback_steady_ns")
        report["axes"][axis] = {
            "speed_rad_s":finite_stats(array(data,p+"model_velocity", np.nan)[active]),
            "tracking_error":finite_stats(array(data,p+"tracking_error",np.nan)[active]),
            "reported_torque_nm":finite_stats(array(data,m+"torque",np.nan)[active]),
            "saturated_fraction":float(np.mean(array(data,p+"saturated")[active])),
            "feedback_age_ms":finite_stats((ns[active]-feedback_ns[active])*1e-6)}
    for (run, stage), frame in data.loc[active].groupby([S+"run_id", S+"stage"], sort=False):
        row={"run":int(run), "stage":int(stage), "rows":len(frame),
             "mode":int(frame[S+"mode"].iloc[0]), "side":int(frame[S+"side"].iloc[0]),
             "waveform":int(frame[S+"waveform"].iloc[0]),
             "validation":int(frame[S+"validation"].iloc[0]),
             "wall_s":float((frame[S+"control_steady_ns"].iloc[-1]-frame[S+"control_steady_ns"].iloc[0])*1e-9)}
        if S+"stage_elapsed_s" in frame:
            row.update(elapsed_start=float(frame[S+"stage_elapsed_s"].min()),
                       elapsed_end=float(frame[S+"stage_elapsed_s"].max()),
                       duration_s=float(frame[S+"stage_duration_s"].iloc[0]))
            row["complete"]=(row["elapsed_start"] <= .03 and
                             row["elapsed_end"] >= row["duration_s"]-.03)
        row["beta_ref_deg"] = finite_stats(np.rad2deg(array(frame,S+"beta_reference",np.nan)))
        for axis in (AXES[4:] if report["experiment"] == [1] else AXES[:2]):
            p,m=S+axis+"/", "/wheel_leg/"+axis+"/"
            row[axis] = {"target_speed":finite_stats(array(frame,p+"reference_velocity",np.nan)),
                         "speed":finite_stats(array(frame,p+"model_velocity",np.nan)),
                         "torque":finite_stats(array(frame,m+"torque",np.nan)),
                         "saturated_fraction":float(np.mean(array(frame,p+"saturated")))}
        report["stages"].append(row)
    return report


def concatenate_observations(rows):
    return {key:np.concatenate([r[key] for r in rows]) for key in rows[0]}


def identify_wheels(directory,output,cache):
    from sweep_archive_models import (stage_frames,uniform_series,integral_observations,
                                     fit_wheel,validate_wheel,fit_current_delay)
    observations={axis:[] for axis in AXES[4:]}
    trajectories={axis:[] for axis in AXES[4:]}
    delays={axis:{"low_speed":[],"high_speed":[]} for axis in AXES[4:]}
    for filename in ("wheel_leg_轮子低速辨识数据.csv","wheel_leg_轮子高速辨识数据.csv"):
        data,info=load_source(directory/filename,cache)
        high=S+"wheel_high_speed" in data
        for run,stage,complete,frame in stage_frames(data):
            if not complete:continue
            waveform=int(frame[S+"waveform"].iloc[0])
            validation=bool(frame[S+"validation"].iloc[0])
            selected=[axis for axis in AXES[4:] if np.max(np.abs(array(frame,S+axis+"/reference_velocity")))>.05]
            # High-speed v1 has 15 stages per single-wheel direction. Confirmed
            # revision and zero effort distinguish free releases from a zero-speed servo.
            coast=high and stage>0 and stage%15 in (3,14)
            if coast:
                selected=[AXES[4+(stage-1)//30]]
            if len(selected)!=1:continue
            axis=selected[0]
            series=uniform_series(frame,[axis])
            if series is None:continue
            heldout=validation or (coast and stage%15==14)
            series.update(source=filename,run=run,stage=stage,heldout=heldout,coast=coast)
            trajectories[axis].append(series)
            observation=integral_observations(series)
            observation["heldout"]=np.full(len(observation["t"]),heldout)
            observations[axis].append(observation)
            if (high and waveform==9) or (not high and waveform in (6,8)):
                # Small frame subsets suffice for command/current delay and avoid
                # retaining the entire archive while fitting both wheels.
                columns=[x for x in frame if x.startswith(S+axis+"/") or x.startswith("/wheel_leg/"+axis+"/")
                         or x in (S+"control_steady_ns",)]
                delays[axis]["high_speed" if high else "low_speed"].append((frame[columns].copy(),validation))
        del data
    report={"real2sim_ready":False,"auto_applied":False,"wheels":[]}
    for axis in AXES[4:]:
        print("Fitting "+axis,flush=True)
        model=fit_wheel(concatenate_observations(observations[axis]))
        if "parameters" in model:
            model["holdout_rollout"]=validate_wheel(model,trajectories[axis])
            model["accepted"] &= model["holdout_rollout"]["accepted"]
        model["axis"]=axis
        model["low_speed_current_delay"]=fit_current_delay(delays[axis]["low_speed"],axis)
        model["high_speed_current_delay"]=fit_current_delay(delays[axis]["high_speed"],axis,40)
        report["wheels"].append(model)
        write_json(output/"wheel_models.json",report)
        print(json.dumps(clean_json(model),ensure_ascii=False),flush=True)


def identify_legs(directory,output,cache):
    from sweep_archive_models import (stage_frames,uniform_series,integral_observations,
                                     fit_leg_local,fit_current_delay)
    from analyze_joint_sweep import validate_rollouts
    data,info=load_source(directory/"左腿关节不同夹角扫频.csv",cache)
    grouped={50:[],75:[],100:[]}; series_by_point={k:[] for k in grouped}
    delay_frames=[]
    axes=AXES[:2]
    for run,stage,complete,frame in stage_frames(data):
        if not complete or int(frame[S+"waveform"].iloc[0])!=6:continue
        validation=bool(frame[S+"validation"].iloc[0])
        reference=float(np.rad2deg(frame[S+"beta_reference"]).median())
        center=min(grouped,key=lambda k:abs(k-reference))
        series=uniform_series(frame,axes)
        if series is None:continue
        series.update(run=run,stage=stage,heldout=validation,
                      beta_actual_deg=float(np.rad2deg(frame[S+"left/beta_estimate"]).median()))
        observation=integral_observations(series)
        observation["heldout"]=np.full(len(observation["t"]),validation)
        grouped[center].append(observation)
        series_by_point[center].append(series)
        delay_frames.append((frame,validation))
    report={"source":info,"side":"left","gravity":"omitted: horizontal suspended by operator confirmation",
            "right_leg_identified":False,"real2sim_ready":False,"auto_applied":False,"local_models":[]}
    report["current_delay"]=[fit_current_delay(delay_frames,axis) for axis in axes]
    write_json(output/"leg_models.json",report)
    for center,observations in grouped.items():
        print("Fitting leg reference beta "+str(center),flush=True)
        if not observations:continue
        model=fit_leg_local(concatenate_observations(observations))
        model["reference_beta_deg"]=center
        model["actual_beta_deg"]=float(np.median([s["beta_actual_deg"] for s in series_by_point[center]]))
        rollouts=[]
        if "M" in model:
            for series in series_by_point[center]:
                if not series["heldout"]:continue
                check=validate_rollouts(model,series["t"],series["q"],series["v"],series["tau"],
                    np.ones(len(series["t"]),dtype=bool),np.zeros(len(series["t"])))
                rollouts.append(dict(run=series["run"],stage=series["stage"],**check))
            model["holdout_rollouts"]=rollouts
            model["accepted"] &= bool(rollouts and all(r["accepted"] for r in rollouts))
        report["local_models"].append(model)
        write_json(output/"leg_models.json",report)
        print(json.dumps(clean_json(model),ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--cache",type=Path,default=Path.home()/".cache/rmcs-sweep-archive-20261027")
    parser.add_argument("--mode",choices=("audit","wheels","legs","all"),default="audit")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    args.cache.mkdir(parents=True,exist_ok=True)
    if args.mode in ("wheels","all"):
        identify_wheels(args.directory,args.output,args.cache)
    if args.mode in ("legs","all"):
        identify_legs(args.directory,args.output,args.cache)
    if args.mode!="audit":return
    audits=[]
    for path in sorted(args.directory.glob("*.csv")):
        print("Reading "+path.name,flush=True)
        data,info=load_source(path,args.cache)
        audit=audit_source(data,info)
        audits.append(audit)
        write_json(args.output/"data_audit.json",audits)
        print(json.dumps({k:audit[k] for k in ("file","records","active_records","duration_s",
                   "active_modes","revisions","validation_records","completed_records")},ensure_ascii=False),flush=True)
        del data


if __name__ == "__main__":
    main()
