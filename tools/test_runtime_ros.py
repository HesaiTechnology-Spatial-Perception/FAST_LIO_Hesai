#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Integration regressions. Run only in an isolated ROS domain/master, after building."""
import argparse
import copy
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import yaml

from mapping_control import Client, verify_saved_maps, validate_drain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros', type=int, choices=(1, 2), required=True)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--cases', nargs='+', choices=('dense_pause_save','rollback','duplicate','nonfinite','schema','imu_gap','skipped_tail'),
                        default=['dense_pause_save','rollback','duplicate','nonfinite','schema','imu_gap','skipped_tail'])
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    if args.ros == 2:
        from sensor_msgs.msg import Imu, PointCloud2, PointField
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    else:
        from sensor_msgs.msg import Imu, PointCloud2, PointField
    client = Client(args.ros)
    previews = []
    if args.ros == 2:
        sensor_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_pub = client.node.create_publisher(Imu, '/lidar_imu', sensor_qos)
        cloud_pub = client.node.create_publisher(PointCloud2, '/lidar_points', 10)
        preview_sub = client.node.create_subscription(PointCloud2, '/Laser_map',
            lambda msg: previews.append((msg.header.stamp.sec, msg.header.stamp.nanosec, msg.width)), QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    else:
        imu_pub = client.api.Publisher('/lidar_imu', Imu, queue_size=1000)
        cloud_pub = client.api.Publisher('/lidar_points', PointCloud2, queue_size=10)
        preview_sub = None

    def stamp(msg, value):
        sec = int(value); nsec = round((value-sec)*1e9)
        if args.ros == 2:
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nsec
        else:
            msg.header.stamp = client.api.Time(sec, nsec)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if args.ros == 2: client.api.spin_once(client.node, timeout_sec=0.02)
            else: time.sleep(0.02)

    def imu(value, invalid=False):
        msg = Imu(); stamp(msg, value)
        msg.linear_acceleration.z = math.nan if invalid else 9.80665
        imu_pub.publish(msg)

    dtype = np.dtype({'names': ['x','y','z','intensity','ring','timestamp'],
        'formats': ['<f4','<f4','<f4','<f4','<u2','<f8'],
        'offsets': [0,4,8,12,16,24], 'itemsize': 32})
    points = np.zeros(112000, dtype=dtype)
    points['x'] = (np.arange(len(points)) % 350)*0.1-17.5
    points['y'] = (np.arange(len(points)) // 350)*0.1-16
    points['z'] = 1.0; points['intensity'] = 50; points['ring'] = np.arange(len(points)) % 16

    def cloud(value, bad_schema=False, sparse=False):
        msg = PointCloud2(); stamp(msg, value); msg.header.frame_id = 'lidar'
        msg.width = len(points); msg.height = 1; msg.point_step = 32; msg.row_step = 32*len(points)
        msg.is_dense = True; msg.is_bigendian = False
        for name, offset, kind in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('timestamp',24,8)]:
            field = PointField(); field.name=name; field.offset=offset
            field.datatype=7 if bad_schema and name=='timestamp' else kind; field.count=1
            msg.fields.append(field)
        points['timestamp'] = value+np.linspace(0,0.18,len(points))
        msg.data=points.tobytes()
        if bad_schema:
            msg.width=1; msg.row_step=32; msg.data=msg.data[:32]
        if sparse:
            msg.width=3; msg.row_step=96; msg.data=msg.data[:96]
        cloud_pub.publish(msg)

    source = yaml.safe_load(Path(args.config).read_text())
    reports = []
    process = None
    try:
        for case in args.cases:
            folder = output/case; folder.mkdir()
            if args.ros == 2:
                client.node.destroy_publisher(imu_pub)
                # Reliable delivery isolates the large-frame memory test from UDP
                # fragmentation loss. Error cases explicitly exercise Best Effort IMU.
                qos=QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE
                    if case in ('dense_pause_save','imu_gap','skipped_tail') else ReliabilityPolicy.BEST_EFFORT)
                imu_pub=client.node.create_publisher(Imu, '/lidar_imu', qos)
            config = copy.deepcopy(source)
            p = config if args.ros == 1 else config['/**']['ros__parameters']
            p.setdefault('common', {}).update(run_id=case, log_dir=str(folder), imu_gyr_unit='rad',
                reliable_input=case in ('dense_pause_save','imu_gap','skipped_tail'),
                lid_topic='/lidar_points', imu_topic='/lidar_imu', time_offset_lidar_to_imu=0.0)
            p.update(filter_size_surf=0.04, filter_size_map=0.1, point_filter_num=1,
                     feature_extract_enable=False, map_file_path=str(folder/'map.pcd'))
            p['preprocess']['blind']=0.1
            p['mapping'].update(imu_init_delay=0.0, imu_init_duration=0.0,
                                imu_init_max_gyr_norm=0.0, extrinsic_est_en=False)
            p.setdefault('publish', {}).update(map_en=True, scan_publish_en=False)
            p.setdefault('pcd_save', {}).update(pcd_save_en=case=='dense_pause_save',mode='all',leaf_size=0.1,interval=-1)
            p.setdefault('trajectory_save', {})['tum_en']=False
            config_path=folder/'config.yaml'; config_path.write_text(yaml.safe_dump(config))
            if args.ros == 1:
                subprocess.run(['rosparam','load',str(config_path)], check=True)
                command=[args.executable]
            else:
                command=[args.executable,'--ros-args','--params-file',str(config_path)]
            with (folder/'mapping.log').open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                client.status(20,case)
                deadline=time.monotonic()+10
                while time.monotonic()<deadline:
                    subscribers=(imu_pub.get_subscription_count() if args.ros==2 else imu_pub.get_num_connections())
                    clouds=(cloud_pub.get_subscription_count() if args.ros==2 else cloud_pub.get_num_connections())
                    if subscribers and clouds: break
                    spin(0.1)
                assert subscribers and clouds, 'Publisher/subscriber discovery failed'
                spin(0.3)
                if case in ('dense_pause_save','imu_gap','skipped_tail'):
                    for frame in range(8):
                        t=1000+frame*0.2
                        for index in range(40):
                            imu(t+index*0.005); spin(0.002)
                        cloud(t)
                        deadline=time.monotonic()+20
                        while True:
                            status=client.status(10,case)
                            if status['synchronized_frames']>=frame+1: break
                            if time.monotonic()>=deadline: raise TimeoutError(str(status))
                            spin(0.1)
                    assert status['received_imu']==320, status
                    assert status['registered_frames']>0, status
                    assert status['last_downsampled_points']>100000, status
                    if case == 'imu_gap':
                        imu(1005.0)
                        expected=3; detail={'before_gap':status}
                    elif case == 'skipped_tail':
                        for index in range(40):
                            imu(1001.6+index*0.005); spin(0.002)
                        cloud(1001.6,sparse=True)
                        deadline=time.monotonic()+20
                        while True:
                            status=client.status(5,case)
                            if status['synchronized_frames']==9 and not status['processing_frame']: break
                            if time.monotonic()>=deadline: raise TimeoutError(str(status))
                            spin(0.1)
                        assert status['skipped_frames']==1,status
                        try:
                            validate_drain(status,{'lidar_count':9,'imu_count':360})
                            raise AssertionError('Skipped tail incorrectly accepted as complete')
                        except ValueError as error:
                            assert 'skipped' in str(error),str(error)
                        process.send_signal(signal.SIGINT); expected=0
                        detail={'status':status,'incomplete_rejected':True}
                    else:
                        response=client.call('/map_save',30); assert response.success,response.message
                        saved=json.loads(response.message); verify_saved_maps(saved)
                        mtimes=[Path(f['path']).stat().st_mtime_ns for f in saved['files']]
                        spin(2.2)
                        response2=client.call('/map_save',30); assert response2.success,response2.message
                        assert json.loads(response2.message)==saved, 'Repeated save changed accumulated counts'
                        assert mtimes==[Path(f['path']).stat().st_mtime_ns for f in saved['files']], 'Repeated save rewrote files'
                        if args.ros==2:
                            assert previews, 'No map preview received'
                            assert len({(s,n) for s,n,w in previews})==len(previews), 'Preview republished stale frame'
                        process.send_signal(signal.SIGINT); expected=0
                        detail={'status':status,'maps':saved,'preview_messages':len(previews)}
                else:
                    if case=='schema': cloud(2000,bad_schema=True)
                    else:
                        imu(2000,invalid=case=='nonfinite'); spin(0.1)
                        if case=='rollback': imu(1999)
                        elif case=='duplicate': imu(2000)
                    expected=3; detail={}
                code=process.wait(timeout=15)
                assert code==expected,(case,code,(folder/'mapping.log').read_text()[-1500:])
                final=json.loads((folder/'node_status.json').read_text())
                assert final['exit_code']==expected,final
                reports.append(dict(case=case,exit_code=code,final=final,**detail))
                (output/'result.json').write_text(json.dumps(reports,indent=2)+'\n')
                process=None
                spin(0.3)
                print(case,'passed',flush=True)
        (output/'result.json').write_text(json.dumps(reports,indent=2)+'\n')
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        client.close()
    print('ROS',args.ros,'runtime regressions passed:',output/'result.json')


if __name__=='__main__':
    main()
