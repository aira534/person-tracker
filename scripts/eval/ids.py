#!/usr/bin/env python3
"""Per-id lifetime, birth/death position and % of scans with lidar.
usage: python3 scripts/eval/ids.py <recording>"""
import sys, math, rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseArray
from vision_msgs.msg import Detection3DArray
from person_tracker_interfaces.msg import DetectedEntities
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
T = {'/lidar_objects': PoseArray, '/detections_3d_odom': Detection3DArray, '/tracked_entities': DetectedEntities}
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
ids, cams, lid = {}, [], []
while r.has_next():
    tp, data, _ = r.read_next()
    m = deserialize_message(data, T[tp])
    t = st(m.header)
    if tp == '/tracked_entities':
        for e in m.entities:
            d = ids.setdefault(e.id, {'t0': t, 't1': t, 'p0': (e.position.x, e.position.y), 'p1': None, 'lid': 0, 'n': 0})
            d['t1'] = t; d['p1'] = (e.position.x, e.position.y); d['n'] += 1; d['lid'] += e.misses == 0
    elif tp == '/detections_3d_odom':
        cams.append((t, [(d.bbox.center.position.x, d.bbox.center.position.y) for d in m.detections]))
    else:
        lid.append(t)
t0 = min(lid)
print(' id  nace   muere  vida   pos_inicial      pos_final     %scans_con_lidar')
for i, d in sorted(ids.items()):
    print(f'{i:3} {d["t0"]-t0:5.2f}  {d["t1"]-t0:5.2f}  {d["t1"]-d["t0"]:4.1f}  ({d["p0"][0]:5.2f},{d["p0"][1]:5.2f})  ({d["p1"][0]:5.2f},{d["p1"][1]:5.2f})  {100*d["lid"]/d["n"]:4.0f}%')
from collections import Counter
print('\npersonas por detección de cámara:', sorted(Counter(len(c) for _, c in cams).items()), f'(mensajes: {len(cams)})')
