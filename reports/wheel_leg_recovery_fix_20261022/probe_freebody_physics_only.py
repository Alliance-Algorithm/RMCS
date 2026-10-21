#!/usr/bin/env python3
"""Bounded free-body Isaac replay, native script until failure or first RL event.

No guides, repeated pose writes, surrogate balance PD, or training.
With --policy, the official ONNX actor owns all axes after handover. Otherwise
all motor efforts are zero after the script ends; that tail is passive.
"""
import argparse
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback
import numpy as np
from scipy.spatial.transform import Rotation

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=4.)
    parser.add_argument('--cases',nargs='+')
    parser.add_argument('--policy',action='store_true')
    args=parser.parse_args()
    assert 1. <= args.seconds <= 8.
    root=Path(__file__).resolve().parents[2]
    training=Path('/home/noir/Documents/workspace/isaac_wheeled_rl_train')
    core=root/'rmcs_ws/src/rmcs_core'
    bundle=training/'model/纯底盘_v7_flat_base107/urdf'
    sys.path.insert(0,str(training/'scripts'))
    from recovery_static_targets import RecoveryStaticTargetBuilder
    builder=RecoveryStaticTargetBuilder(bundle,sha(bundle/'manifest.json'))
    spec,manifest,fit=builder.spec,builder.manifest,builder.fit
    active=['L_joint1','LL_joint1','R_joint1','RR_joint1','L_joint3','R_joint3']
    cases=[]
    for name,knee,roll in [('standing_50',50.,None),('folded_40_5',40.5,None),('roll_18',50.,18.)]:
        pose=builder.balanced_pose(knee)
        rotation=Rotation.from_euler('x',pose['root_roll_rad'] if roll is None else math.radians(roll))
        cases.append((name,pose['q'],rotation))
    pose=builder.target(.305)
    # target() returns a static pose and separately compensated actuator targets.
    q=pose['full_joint_positions']
    cases.append(('standing_305',q,Rotation.from_euler('x',pose['root_roll_rad'])))
    source=Path('/home/noir/wheel_leg_rl_2026-10-21_17-56-31.csv')
    # The first of the last pair reports 107.0038 deg, slightly beyond the
    # immutable 107 deg rigid stop. Use the second (104.62 deg), without clipping.
    indices={3577:'csv_first_upright',6409:'csv_upside_down',19349:'csv_latest_asymmetric'}
    with source.open() as handle:
        rows={int(row['index']):row for row in csv.DictReader(handle) if int(row['index']) in indices}
    for index,name in indices.items():
        row=rows[index]
        names=['left_hip_joint','left_knee_joint','right_hip_joint','right_knee_joint']
        desired=np.array([float(row['/wheel_leg/rl/model/'+n+'/angle']) for n in names])
        initial=builder.knee_pose(103.,105.)
        start=np.array([initial[n] for n in active[:4]])
        for side in (0,2):
            desired[side:side+2]-=np.round((desired[side]-start[side])/(2*math.pi))*(2*math.pi)
        for f in np.linspace(0.,1.,100)[1:]:
            fixed={n:initial[n] for n in builder.controls}
            fixed.update(zip(active[:4],start+f*(desired-start)))
            initial=builder.solve(fixed,initial)
        g=np.array([float(row['/wheel_leg/rl/input/projected_gravity/'+a]) for a in 'xyz'])
        g/=np.linalg.norm(g)
        rotation=Rotation.from_euler('xyz',[math.atan2(-g[1],-g[2]),math.asin(g[0]),0.])
        assert np.allclose(rotation.inv().apply([0,0,-1]),g,atol=1e-8)
        cases.append((name,initial,rotation))
    if args.cases:
        cases=[case for case in cases if case[0] in args.cases]
        assert len(cases)==len(args.cases)
    args.output.mkdir(parents=True,exist_ok=True)
    library=ctypes.CDLL(str(args.library.resolve()))
    ptr=ctypes.POINTER(ctypes.c_double)
    library.freebody_create.argtypes=[ctypes.c_char_p]
    library.freebody_create.restype=ctypes.c_void_p
    library.freebody_step.argtypes=[ctypes.c_void_p,ptr,ptr,ptr,ptr,ptr,ctypes.c_double,ptr]
    library.freebody_error.restype=ctypes.c_char_p
    policy=None
    nominal=np.array([-.42,.13742282595395358,.42,-.1374155762580851,0.,0.])
    scales=np.array([.25,.25,.25,.25,10.,10.])
    policy_path=root/'rmcs_ws/src/rmcs_rl/models/wheel_leg/recovery_candidate.onnx'
    if args.policy:
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        policy=ort.InferenceSession(str(policy_path),options,providers=['CPUExecutionProvider'])
        assert policy.get_inputs()[0].shape[1]==35
        assert policy.get_modelmeta().custom_metadata_map['leg_target_model_spec_sha256']==sha(bundle/'model_spec.json')
        library.freebody_history.argtypes=[ctypes.c_void_p,ptr,ptr]
        library.freebody_actor_effort.argtypes=[ctypes.c_void_p,ptr,ptr,ptr]
    from isaaclab.app import AppLauncher
    launcher=AppLauncher({'headless':True,'device':'cpu'})
    try:
        import carb.settings
        import torch
        import warp as wp
        import isaaclab.sim as sim_utils
        from isaaclab.actuators import IdealPDActuatorCfg
        from isaaclab.assets import Articulation,ArticulationCfg
        from isaaclab_physx.physics import PhysxCfg,PhysxManager
        from pxr import UsdPhysics,PhysxSchema
        torch.set_num_threads(4)
        dt=.001
        sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(device='cpu',dt=dt,render_interval=100,
            physics=PhysxCfg(solver_type=1,enable_external_forces_every_iteration=True)))
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        floor=sim_utils.GroundPlaneCfg(size=(100.,100.),physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=.8,dynamic_friction=.8,restitution=0.))
        floor.func('/World/Ground',floor)
        floor_paths=[str(p.GetPath()) for p in sim.stage.Traverse()
                     if str(p.GetPath()).startswith('/World/Ground/') and p.HasAPI(UsdPhysics.CollisionAPI)]
        rigs=[]
        springs=manifest['spring_joint_names']
        passive=[n for n in manifest['tree_joint_names'] if n not in active+springs]
        for i,(name,q,rotation) in enumerate(cases):
            path='/World/'+name
            cfg=ArticulationCfg(prim_path=path,spawn=sim_utils.UsdFileCfg(
                usd_path=str(bundle/'robot.usda'),activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=False,
                    enabled_self_collisions=False,solver_position_iteration_count=32,solver_velocity_iteration_count=8)),
                init_state=ArticulationCfg.InitialStateCfg(pos=(i*5.,0.,.7),joint_pos=q,joint_vel={'.*':0.}),
                actuators={
                    'active':IdealPDActuatorCfg(joint_names_expr=active,stiffness=0.,damping=0.,
                        effort_limit=40.,effort_limit_sim=40.,velocity_limit_sim=1000.),
                    'passive':IdealPDActuatorCfg(joint_names_expr=passive,stiffness=0.,damping=.002,
                        effort_limit=100.,effort_limit_sim=100.,velocity_limit_sim=1000.),
                    'gas':IdealPDActuatorCfg(joint_names_expr=springs,stiffness=0.,damping=.002,
                        effort_limit=1000.,effort_limit_sim=1000.,velocity_limit_sim=100.)},
                soft_joint_pos_limit_factor=1.)
            robot=Articulation(cfg)
            for prim in sim.stage.Traverse():
                if str(prim.GetPath()).startswith(path+'/') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
            bounds=builder.collision_bounds(q,rotation.as_matrix())
            z=-min(float(b['lower'][2]) for b in bounds)+.003
            context=library.freebody_create(str(core/'config/wheel_leg').encode())
            if not context:
                raise RuntimeError(library.freebody_error().decode())
            rigs.append(dict(name=name,robot=robot,q0=q,rotation=rotation,z0=z,x0=i*5.,context=context,
                             out=np.zeros(42),samples=[],terminal=None,phases=[],previous_phase=-1,
                             max_pre_tilt=0.,max_tilt=0.,max_closure=0.,prev_velocity=np.zeros(3),
                             gas_clamp_mm=0.,max_torque=0.,hinge_min=1e9,hinge_max=-1e9,decoded=np.zeros(6),action=np.zeros(6),history=np.zeros(6),height_ref=.305,
                             actor_failure=None,post_handover_max_tilt=0.,policy_calls=0))
        sim.reset()
        for r in rigs:
            robot=r['robot']
            r['ids']=[robot.joint_names.index(n) for n in active]
            r['spring_ids']=[robot.joint_names.index(n) for n in springs]
            r['knee_ids']=[robot.joint_names.index(n) for n in ['L_joint2','R_jonit2']]
            r['base_id']=robot.body_names.index('base_link')
            paths=['/World/'+r['name']+'/'+n for n in robot.body_names]
            r['contact']=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                paths,filter_patterns=[floor_paths for _ in paths],max_contact_data_count=4096)
            r['contact_paths']=list(r['contact'].sensor_paths)
            joint=torch.tensor([[r['q0'][n] for n in robot.joint_names]],dtype=torch.float32)
            robot.write_root_link_pose_to_sim_index(root_pose=torch.tensor(
                [[r['x0'],0.,r['z0'],*r['rotation'].as_quat()]],dtype=torch.float32))
            robot.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros(1,6))
            robot.write_joint_position_to_sim_index(position=joint)
            robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joint))
            robot.reset();robot.update(dt)
            r['effort']=torch.zeros_like(joint)
        started=time.monotonic()
        for step in range(round(args.seconds/dt)):
            t=step*dt
            for r in rigs:
                robot=r['robot']
                q=robot.data.joint_pos.torch[0]
                dq=robot.data.joint_vel.torch[0]
                pose=robot.data.body_link_pose_w.torch[0,r['base_id']].numpy()
                velocity=robot.data.body_com_vel_w.torch[0,r['base_id']].numpy().astype(float)
                rotation=Rotation.from_quat(pose[3:])
                g=np.ascontiguousarray(rotation.inv().apply([0.,0.,-1.]))
                gyro=np.ascontiguousarray(rotation.inv().apply(velocity[3:]))
                accel=np.ascontiguousarray(rotation.inv().apply((velocity[:3]-r['prev_velocity'])/dt+[0.,0.,9.81]))
                r['prev_velocity']=velocity[:3].copy()
                tilt=math.degrees(math.acos(np.clip(-g[2],-1.,1.)))
                r['max_tilt']=max(r['max_tilt'],tilt)
                effort=r['effort'];effort.zero_()
                running_actor=policy is not None and r['terminal'] is not None and r['terminal']['kind']=='handover' and r['actor_failure'] is None
                if r['terminal'] is None or running_actor:
                    if r['terminal'] is None:r['max_pre_tilt']=max(r['max_pre_tilt'],tilt)
                    q6=np.ascontiguousarray(q[r['ids']].numpy(),dtype=np.float64)
                    dq6=np.ascontiguousarray(dq[r['ids']].numpy(),dtype=np.float64)
                    code=library.freebody_step(r['context'],q6.ctypes.data_as(ptr),dq6.ctypes.data_as(ptr),
                        g.ctypes.data_as(ptr),gyro.ctypes.data_as(ptr),accel.ctypes.data_as(ptr),dt,
                        r['out'].ctypes.data_as(ptr))
                    out=r['out']
                    if int(out[21])!=r['previous_phase']:
                        event=dict(time_s=t,phase=int(out[21]),failure=int(out[26]),tilt_deg=tilt,
                            knee_deg=out[8:10].tolist(),remaining_rad=float(out[22]),tracking_rad=float(out[23]),
                            helper_valid=bool(out[35]),support=bool(out[37]))
                        r['phases'].append(event);r['previous_phase']=int(out[21])
                        print('PHASE',r['name'],event,flush=True)
                    if code!=1 and r['terminal'] is None:
                        r['terminal']=dict(time_s=t,kind='handover' if code==2 else 'failure',
                                           native_failure=int(out[26]),tilt_deg=tilt)
                    elif code==1:
                        effort[0,r['ids']]=torch.tensor(out[15:21],dtype=torch.float32)
                    if policy is not None:
                        library.freebody_history(r['context'],r['action'].ctypes.data_as(ptr),r['history'].ctypes.data_as(ptr))
                        if step%20==0:
                            if code==2:r['height_ref']+=np.clip(.23-r['height_ref'],-.6666666666666666/50.,.6666666666666666/50.)
                            obs=np.zeros(35)
                            obs[3]=5.*(r['height_ref'] if code==2 else .305)
                            obs[4:7]=gyro*.5;obs[7:10]=g
                            delta=q6[:4]-nominal[:4];obs[10:14]=np.arctan2(np.sin(delta),np.cos(delta))
                            obs[16:22]=dq6*.1;obs[22:28]=r['history'];obs[28]=1.
                            raw=policy.run(None,{'obs':np.clip(obs,-100.,100.).astype(np.float32)[None]})[0][0]
                            assert np.isfinite(raw).all()
                            r['decoded']=np.ascontiguousarray(np.clip(raw*scales,[-.75]*4+[-90.]*2,[.75]*4+[90.]*2))
                            r['action']=np.ascontiguousarray(r['decoded']/scales)
                            r['policy_calls']+=1
                        if code==2:
                            r['post_handover_max_tilt']=max(r['post_handover_max_tilt'],tilt)
                            hinge=np.array([out[32]-out[31],out[33]-out[34]])
                            if tilt>45. or hinge.min()<-.170638855 or hinge.max()>1.4189088544:
                                r['actor_failure']=dict(time_s=t,tilt_deg=tilt,hinge=hinge.tolist(),reason='native tilt/hinge stop')
                            else:
                                tau=np.zeros(6)
                                library.freebody_actor_effort(r['context'],dq6.ctypes.data_as(ptr),r['decoded'].ctypes.data_as(ptr),tau.ctypes.data_as(ptr))
                                effort[0,r['ids']]=torch.tensor(tau,dtype=torch.float32)
                    r['max_torque']=max(r['max_torque'],float(np.abs(out[15:19]).max()))
                    hinge=np.array([out[32]-out[31],out[33]-out[34]])
                    r['hinge_min']=min(r['hinge_min'],float(hinge.min()))
                    r['hinge_max']=max(r['hinge_max'],float(hinge.max()))
                compression=np.array([spec['spring_binding'][n]['compression_at_q_zero_m'] for n in springs])-q[r['spring_ids']].numpy()
                r['gas_clamp_mm']=max(r['gas_clamp_mm'],float(np.maximum(compression-.072,0.).max()*1000))
                force=np.polynomial.polynomial.polyval(np.clip(compression,0.,.072)/.08,fit['monomial_coefficients_n'])
                effort[0,r['spring_ids']]=torch.tensor(force,dtype=torch.float32)
                robot.set_joint_effort_target_index(target=effort);robot.write_data_to_sim()
                if step%10==0 or (r['terminal'] and abs(t-r['terminal']['time_s'])<dt/2):
                    poses=robot.data.body_link_pose_w.torch[0].numpy().astype(float)
                    def point(body,local):
                        pose=poses[robot.body_names.index(body)]
                        return pose[:3]+Rotation.from_quat(pose[3:]).apply(local)
                    closure=max(np.linalg.norm(point(c['body0'],c['local_pos0_m'])-point(c['body1'],c['local_pos1_m'])) for c in spec['constraints'])
                    r['max_closure']=max(r['max_closure'],float(closure))
                    forces=wp.to_torch(r['contact'].get_contact_force_matrix(dt=dt)).numpy().sum(axis=1)
                    wheels=[r['contact_paths'].index('/World/'+r['name']+'/'+side+'_link3') for side in ['L','R']]
                    r['samples'].append(dict(time_s=t,tilt_deg=tilt,roll_pitch_yaw_deg=rotation.as_euler('xyz',degrees=True).tolist(),
                        base_xyz_m=pose[:3].tolist(),gravity=g.tolist(),gyro=gyro.tolist(),acceleration=accel.tolist(),
                        knee_deg=[math.degrees(float(q[r['knee_ids'][0]])+builder.knee_zero_rad),
                                  math.degrees(-float(q[r['knee_ids'][1]])+builder.knee_zero_rad)],
                        q=q[r['ids']].numpy().tolist(),dq=dq[r['ids']].numpy().tolist(),
                        native_output=r['out'].tolist(),motor_effort=effort[0,r['ids']].numpy().tolist(),
                        wheel_support_n=forces[wheels,2].tolist(),closure_gap_m=float(closure),
                        post_terminal=r['terminal'] is not None))
            sim.step(render=False)
            for r in rigs:r['robot'].update(dt)
            if step%1000==0:print('STEP',step,'wall_s',round(time.monotonic()-started,1),flush=True)
        result={
            'scope':'Unrestrained 6-DOF base and ground contacts, compiled C++ tuck/FSM/native support observer and dynamic simulated IMU. No ROS worker timing, CAN delays, hardware acceptance, or self collision clearance validation. See policy_scope for actual actor execution.',
            'policy_sha256':sha(policy_path) if policy is not None else None,
            'policy_scope':'Official ONNX, 35-D YAML order/scales, 50 Hz shadow inference, native action winding/PD/effective history, 1 kHz physics. No ROS worker timing or CAN.' if policy else 'No actor',
            'fixture':'none','isaac_sim_version':'6.0.0.1','dt':dt,'duration_s':args.seconds,
            'native_sources_sha256':{str(p.relative_to(root)):sha(p) for p in (core/'src/controller/chassis/wheel_leg').glob('*.cpp')},
            'source_csv_sha256':sha(source),'library_sha256':sha(args.library),'script_sha256':sha(Path(__file__)),
            'model_spec_sha256':sha(bundle/'model_spec.json'),'manifest_sha256':sha(bundle/'manifest.json'),
            'results':[]}
        for r in rigs:
            summary={k:r[k] for k in ['name','z0','terminal','phases','max_pre_tilt','max_tilt','max_closure',
                                           'gas_clamp_mm','max_torque','hinge_min','hinge_max','actor_failure','post_handover_max_tilt','policy_calls']}
            final=r['samples'][-1]
            summary['final_tilt_deg']=final['tilt_deg']
            summary['final_knee_deg']=final['knee_deg']
            summary['final_wheel_support_n']=final['wheel_support_n']
            summary['max_applied_leg_effort_nm']=max(max(abs(v) for v in a['motor_effort'][:4]) for a in r['samples'])
            summary['max_applied_wheel_effort_nm']=max(max(abs(v) for v in a['motor_effort'][4:]) for a in r['samples'])
            if r['terminal'] is not None:
                hand=min(r['samples'],key=lambda a:abs(a['time_s']-r['terminal']['time_s']))
                summary['terminal_knee_deg']=hand['native_output'][8:10]
                summary['terminal_body_sagittal_error_mm']=[v*1000. for v in hand['native_output'][10:12]]
            result['results'].append(summary)
            (args.output/(r['name']+'_samples.json')).write_text(json.dumps(r['samples'],indent=2))
        (args.output/'report.json').write_text(json.dumps(result,indent=2))
        print('FREEBODY_REPORT',json.dumps(result),flush=True)
    except BaseException as exc:
        traceback.print_exc()
        (args.output/'failure.json').write_text(json.dumps({'error':repr(exc)},indent=2))
        sys.stdout.flush();sys.stderr.flush()
        raise
    finally:
        launcher.app.close()

if __name__=='__main__':main()
