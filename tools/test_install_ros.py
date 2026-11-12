#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Exercise installed entry points with synthetic data; use an isolated ROS domain/master.

This verifies orchestration and output integrity, not sensor decoding or map accuracy.
No downloaded data or physical sensor is needed. Run after sourcing the install space.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time
import xmlrpc.client

import numpy as np
import yaml


def messages(ros, zero=False):
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                      'offsets': [0, 4, 8, 12, 16, 24], 'itemsize': 32})
    points = np.zeros(4800, dtype=dtype)
    u = (np.arange(1600) % 40) * .25 - 5
    v = (np.arange(1600) // 40) * .25 - 5
    points['x'] = np.concatenate((u, u, np.full(1600, 5)))
    points['y'] = np.concatenate((v, np.full(1600, 5), u))
    points['z'] = np.concatenate((np.full(1600, -2), v, v))
    points['intensity'] = 50
    points['ring'] = np.arange(len(points)) % 16

    def stamp(msg, t):
        sec = int(t); ns = round((t - sec) * 1e9)
        if ros == 2:
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, ns
        else:
            import rospy
            msg.header.stamp = rospy.Time(sec, ns)

    for frame in range(20):
        t = 1000 + frame * .2
        pc = PointCloud2(); stamp(pc, 0 if zero else t)
        pc.header.frame_id = 'lidar'; pc.width = len(points); pc.height = 1
        pc.point_step = 32; pc.row_step = 32 * len(points); pc.is_dense = True
        for name, offset, kind in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                   ('intensity', 12, 7), ('ring', 16, 4), ('timestamp', 24, 8)]:
            f = PointField(); f.name = name; f.offset = offset; f.datatype = kind; f.count = 1
            pc.fields.append(f)
        points['timestamp'] = t + np.linspace(0, .18, len(points))
        pc.data = points.tobytes()
        yield '/lidar_points', pc, t
        for i in range(40):
            ti = t + i * .005
            imu = Imu(); stamp(imu, 0 if zero else ti)
            imu.linear_acceleration.z = 9.80665
            yield '/lidar_imu', imu, ti


def write_bag(ros, path, zero=False):
    if ros == 1:
        import rosbag
        import rospy
        with rosbag.Bag(str(path), 'w') as bag:
            for topic, msg, t in messages(ros, zero):
                bag.write(topic, msg, rospy.Time.from_sec(t))
    else:
        import rosbag2_py
        from rclpy.serialization import serialize_message
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        for topic, typ in [('/lidar_points', 'PointCloud2'), ('/lidar_imu', 'Imu')]:
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type='sensor_msgs/msg/' + typ,
                                                        serialization_format='cdr'))
        for topic, msg, t in messages(ros, zero):
            writer.write(topic, serialize_message(msg), round(t * 1e9))
        del writer


def publish_fixture():
    """ROS1 fake driver: test recorder/master lifecycle, never claim PCAP decoding."""
    import rospy
    from sensor_msgs.msg import PointCloud2, Imu
    rospy.init_node('synthetic_driver')
    cfg = yaml.safe_load(Path(rospy.get_param('~config_path')).read_text())['lidar'][0]
    pubs = {'/lidar_points': rospy.Publisher(cfg['ros']['ros_send_point_cloud_topic'], PointCloud2, queue_size=100),
            '/lidar_imu': rospy.Publisher(cfg['ros']['ros_send_imu_topic'], Imu, queue_size=1000)}
    deadline = time.monotonic() + 15
    while not all(p.get_num_connections() for p in pubs.values()):
        if time.monotonic() > deadline:
            raise TimeoutError('Recorder did not subscribe')
        time.sleep(.05)
    for topic, msg, _ in messages(1):
        pubs[topic].publish(msg)
        time.sleep(.005)
    time.sleep(.5)


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, seconds in [(signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=seconds)
            return
        except subprocess.TimeoutExpired:
            pass
    raise TimeoutError('Test process group did not stop')


def run(cmd, log, expected=0, timeout=120):
    with log.open('w') as stream:
        proc = subprocess.Popen([str(x) for x in cmd], stdout=stream, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            code = proc.wait(timeout=timeout)
        finally:
            stop(proc)
    assert (code != 0 if expected is None else code == expected), (cmd, code, log.read_text()[-3000:])


def fake_driver(root, ros):
    pkg = root/'src/hesai_ros_driver'; pkg.mkdir(parents=True)
    (pkg/'config').mkdir()
    (pkg/'config/config.yaml').write_text('lidar:\n  - driver: {}\n    ros: {}\n')
    (pkg/'package.xml').write_text('<package format="2"><name>hesai_ros_driver</name><version>0.0.0</version>'
        '<description>Synthetic test only</description><maintainer email="test@example.com">Test</maintainer>'
        '<license>GPL-2.0</license></package>')
    if ros == 1:
        setup = root/'devel/setup.bash'; setup.parent.mkdir()
        setup.write_text('export ROS_PACKAGE_PATH=' + shlex.quote(str(root/'src')) + ':"${ROS_PACKAGE_PATH:-}"\n')
        node = pkg/'hesai_ros_driver_node'
    else:
        prefix = root/'install'
        resource = prefix/'share/ament_index/resource_index/packages/hesai_ros_driver'
        resource.parent.mkdir(parents=True); resource.touch()
        setup = prefix/'setup.bash'
        setup.write_text('export AMENT_PREFIX_PATH=' + shlex.quote(str(prefix)) + ':"${AMENT_PREFIX_PATH:-}"\n')
        node = prefix/'lib/hesai_ros_driver/hesai_ros_driver_node'; node.parent.mkdir(parents=True)
    node.write_text('#!/bin/sh\nexit 42\n'); node.chmod(0o755)
    return node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros', type=int, choices=(1, 2), required=True)
    parser.add_argument('--workspace', type=Path, required=True, help='Workspace with install/, no devel/')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    ws, out = args.workspace.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    assert not (ws/'devel').exists(), 'Test must use install space only'
    prefix = ws/'install' if args.ros == 1 else ws/'install/fast_lio'
    share, binary = prefix/'share/fast_lio', prefix/'lib/fast_lio'
    assert (binary/'fastlio_mapping').is_file()
    lookup = ['rospack', 'find', 'fast_lio'] if args.ros == 1 else ['ros2', 'pkg', 'prefix', '--share', 'fast_lio']
    assert Path(subprocess.check_output(lookup, text=True).strip()).resolve() == share
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    master = None
    results = []
    try:
        if args.ros == 1:
            # Allocate a private local master; do not reuse the developer's master.
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
            os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(port)
            os.environ['ROS_HOSTNAME'] = '127.0.0.1'
            node = fake_driver(out/'driver', 1)
            node.write_text('#!/bin/sh\nexec python3 ' + shlex.quote(str(Path(__file__).resolve())) + ' --publish-fixture "$@"\n')
            capture = out/'fixture.pcap'; capture.write_bytes(b'Synthetic driver fixture; not a real PCAP')
            correction = out/'fixture.csv'; correction.write_text('Channel,Elevation,Azimuth\n1,0,0\n')
            cmd = [binary/'pcap_to_rosbag_ros1.sh', capture, '--model', 'jt16', '--driver-ws', out/'driver',
                   '--correction', correction]
            def master_up():
                try:
                    return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/ci')[0] == 1
                except OSError:
                    return False
            run(cmd + ['--output', out/'cold.bag'], out/'cold.log', timeout=75)
            assert not master_up(), 'Converter leaked its owned master'
            results.append('cold_master_conversion')
            with (out/'master.log').open('w') as log:
                master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            deadline = time.monotonic() + 15
            while not master_up():
                assert time.monotonic() < deadline, 'Master startup timed out'
                time.sleep(.1)
            run(cmd + ['--output', out/'warm.bag'], out/'warm.log', timeout=75)
            assert master_up() and master.poll() is None, 'Converter stopped an existing master'
            results.append('existing_master_preserved')
            node.write_text('#!/bin/sh\nexit 42\n')
        else:
            fake_driver(out/'driver', 2)
        bag = out/('valid.bag' if args.ros == 1 else 'valid')
        write_bag(args.ros, bag)
        base = [binary/'run_fastlio.sh', 'jt16']
        opts = ['--ros', args.ros, '--fastlio-ws', ws, '--no-rviz', '--imu-gyr-unit', 'rad',
                '--drain-timeout', '20', '--save-timeout', '30']
        run(base + ['bag', bag] + opts + ['--save-map', out/'map', '--log-dir', out/'valid_run'], out/'valid.log')
        result = json.loads((out/'valid_run/result.json').read_text())
        assert result['exit_code'] == 0 and result['input_complete'] and result['map_saved'], result
        status = result['drain']['status']
        assert status['received_lidar'] == 20 and status['received_imu'] == 800, status
        assert status['registered_frames'] > 0 and status['skipped_frames'] == 0, status
        assert status['registered_frames'] + status['startup_frames'] == 20, status
        saved = result['map_save']['maps']
        assert saved['mode'] == '1hz' and saved['sampled_frames'] > 1, saved
        assert saved['files'] and all(f['points'] > 0 and Path(f['path']).is_file() for f in saved['files']), saved
        results.append('installed_bag_complete_and_1hz_saved')
        bad = out/('invalid.bag' if args.ros == 1 else 'invalid')
        write_bag(args.ros, bad, zero=True)
        run(base + ['bag', bad] + opts + ['--log-dir', out/'invalid_run'], out/'invalid.log', expected=None)
        result = json.loads((out/'invalid_run/result.json').read_text())
        assert not result['input_complete'] and not result['map_saved'] and 'node_status' not in result, result
        assert 'timestamp' in (out/'invalid.log').read_text().lower()
        results.append('all_zero_timestamps_rejected_before_launch')
        run(base + ['live'] + opts + ['--driver-ws', out/'driver', '--log-dir', out/'live_run'],
            out/'live.log', expected=42)
        result = json.loads((out/'live_run/result.json').read_text())
        assert result['exit_code'] == 42 and not result['input_complete'], result
        results.append('live_driver_exit_propagated')
        (out/'result.json').write_text(json.dumps({'ros': args.ros, 'passed': results}, indent=2) + '\n')
        print('Installed integration cases passed:', results, flush=True)
    finally:
        stop(master)


if __name__ == '__main__':
    import sys
    if '--publish-fixture' in sys.argv:
        sys.argv.remove('--publish-fixture')
        publish_fixture()
    else:
        main()
