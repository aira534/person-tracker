#!/usr/bin/env python3
"""Rotate the entire robot reference frame by 180 degrees and shift scan data.

- Modifies /tf and /tf_static: rotates transforms by 180 deg about z.
- Modifies /odom: rotates pose by 180 deg.
- Modifies /scan and /scan_filtered: rotates range data 180 deg to preserve physical world points.
- Original bag is read-only; writes a new bag.

Usage:
  python3 fix_complete_bag.py bag_original bag_listo
"""
import argparse
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

Q180 = (0.0, 0.0, 1.0, 0.0)  # Quaternion for yaw = 180 deg


def qmul(a, b):
  ax, ay, az, aw = a
  bx, by, bz, bw = b
  return (aw * bx + ax * bw + ay * bz - az * by,
          aw * by - ax * bz + ay * bw + az * bx,
          aw * bz + ax * by - ay * bx + az * bw,
          aw * bw - ax * bx - ay * by - az * bz)


def rotate_pose_180(pos, q):
  q.x, q.y, q.z, q.w = qmul(Q180, (q.x, q.y, q.z, q.w))
  pos.x, pos.y = -pos.x, -pos.y


def shift_scan_180(msg):
  # Roll the ranges and intensities array by half length (180 deg)
  n = len(msg.ranges)
  if n > 0:
    half = n // 2
    msg.ranges = list(msg.ranges[half:]) + list(msg.ranges[:half])
  if len(msg.intensities) == n and n > 0:
    half = n // 2
    msg.intensities = list(msg.intensities[half:]) + list(msg.intensities[:half])


def main():
  p = argparse.ArgumentParser(
      description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('input_bag', help='input original bag path')
  p.add_argument('output_bag', help='output bag path')
  p.add_argument('--storage', default=None)
  args = p.parse_args()

  if os.path.exists(args.output_bag):
    sys.exit(f'Error: {args.output_bag} already exists.')

  storage = args.storage
  if storage is None:
    info = rosbag2_py.Info().read_metadata(args.input_bag, '')
    storage = info.storage_identifier or 'sqlite3'

  reader = rosbag2_py.SequentialReader()
  reader.open(rosbag2_py.StorageOptions(uri=args.input_bag, storage_id=storage),
              rosbag2_py.ConverterOptions('cdr', 'cdr'))
  writer = rosbag2_py.SequentialWriter()
  writer.open(rosbag2_py.StorageOptions(uri=args.output_bag, storage_id=storage),
              rosbag2_py.ConverterOptions('cdr', 'cdr'))

  types = {}
  for t in reader.get_all_topics_and_types():
    writer.create_topic(t)
    types[t.name] = get_message(t.type)

  n_total = 0
  while reader.has_next():
    topic, data, ts = reader.read_next()
    n_total += 1

    if topic in ('/tf', '/tf_static'):
      m = deserialize_message(data, types[topic])
      for tr in m.transforms:
        if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
          # Rotate base_link orientation relative to odom
          tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = qmul(
              (tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w),
              Q180)
      data = serialize_message(m)

    elif topic == '/odom':
      m = deserialize_message(data, types[topic])
      m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = qmul(
          (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w),
          Q180)
      data = serialize_message(m)

    elif topic in ('/scan', '/scan_filtered'):
      m = deserialize_message(data, types[topic])
      shift_scan_180(m)
      data = serialize_message(m)

    writer.write(topic, data, ts)

  del writer, reader
  print(f'Processed {n_total} messages into {args.output_bag}')


if __name__ == '__main__':
  main()
  