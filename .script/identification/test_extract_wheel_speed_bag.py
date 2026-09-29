import numpy as np

from extract_wheel_speed_bag import audit


def synthetic():
    n=4
    a={key: np.zeros((n, size)) for key,size in (
        ('dq_api',6),('dm_status',4),('tau_preclip_api',6),('tau_cmd_api',6),
        ('tau_frame_api',6),('wheel_velocity_target_api',2),('tx_can_bus',6),('tx_can_id',6))}
    a.update(phase=np.array([2,2,2,3]),segment_id=np.array([0,0,1,-1]),
        failure_reason=np.zeros(n),dropped_samples=np.zeros(n),repetition_id=np.zeros(n),
        actuation_scope=np.full(n,3),tick=np.arange(n,dtype=np.uint64),
        control_steady_ns=np.arange(n,dtype=np.uint64)*1000000+1000000000,
        tx_frame_bytes=np.zeros((n,48),dtype=np.uint8))
    a['feedback_steady_ns']=np.tile(a['control_steady_ns'][:,None],(1,6))-100000
    a['tx_can_id'][:,4:6]=0x200
    a['wheel_velocity_target_api'][:2,0]=2.
    a['dq_api'][:2,4]=1.
    a['tau_preclip_api'][:2,4]=a['tau_cmd_api'][:2,4]=.2
    scale=20*15.8*.3*187/3591
    count=round(-.2/scale*16384)
    for start in (32,40):
        a['tx_frame_bytes'][:2,start]=(count&0xffff)>>8
        a['tx_frame_bytes'][:2,start+1]=count&0xff
    a['tau_frame_api'][:2,4]=-count*scale/16384
    params={'wheel_velocity_kp':.2,'wheel_torque_cap':scale}
    manifest={'segments':[{'id':i,'coast':bool(i),'axis_scale':[1.,0.],
        'label':'test','validation':bool(i)} for i in (0,1)]}
    return a,params,manifest


def test_audits_signed_c620_encoding_and_coast():
    a,p,m=synthetic()
    r=audit(a,p,m)
    assert r['status']=='complete_for_analysis'
    assert r['p_algebra_max_error_nm']==0.
    assert r['raw_current_encoding_max_error_nm'] < .000151
    assert r['segments'][1]['mode']=='zero_current'


def test_rejects_hidden_inactive_wheel_servo_and_raw_packet_corruption():
    a,p,m=synthetic()
    a['tau_cmd_api'][0,5]=.1
    assert 'P/current limit algebra mismatch' in audit(a,p,m)['reasons']
    a,p,m=synthetic()
    a['tx_frame_bytes'][0,32] ^= 0x40
    assert any('raw C620' in s for s in audit(a,p,m)['reasons'])


def test_clock_gap_is_detected_even_with_contiguous_tick_numbers():
    a,p,m=synthetic()
    a['control_steady_ns'][2:]+=9000000
    r=audit(a,p,m)
    assert r['tick_gap_count']==0
    assert r['sample_interval_max_ms']==10.
    assert any('clock gaps' in s for s in r['reasons'])
