#!/bin/bash
# Runs the full pipeline (YOLO + person_tracker) on each bag and records the tracker topics.
# Uses ROS_DOMAIN_ID=77 so it doesn't interfere with anything else running.
#
# usage: scripts/eval/run_bags.sh <output_dir> [bag1 bag2 ...]
#   (without bags: all of ~/TFG_ws/bags/corrected_bags)
# At the end it prints a table of IDs per bag.
set -m
WS=$HOME/TFG_ws
OUT=$(realpath -m "$1"); shift
BAGS=${@:-$(ls $WS/bags/corrected_bags)}
export ROS_DOMAIN_ID=77
cd $WS
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
mkdir -p $OUT
for B in $BAGS; do
  [ -f $WS/bags/corrected_bags/$B/metadata.yaml ] || continue
  rm -rf $OUT/$B
  ros2 launch person_tracker person_tracker.launch.py > $OUT/$B.launch.log 2>&1 &
  LP=$!
  sleep 15   # YOLO takes a while to load
  ros2 bag record -o $OUT/$B /lidar_objects /detections_3d_odom /tracked_entities > /dev/null 2>&1 &
  RP=$!
  sleep 2
  ros2 bag play bags/corrected_bags/$B --clock > /dev/null 2>&1
  sleep 3
  kill -INT -$RP; wait $RP
  kill -INT -$LP
  for i in $(seq 20); do kill -0 -$LP 2>/dev/null || break; sleep 0.5; done
  kill -KILL -$LP 2>/dev/null
  [ -f $OUT/$B/metadata.yaml ] && echo "ok $B" || echo "FALLO $B"
done

# table of IDs per bag: distinct ids the tracker created (ideal = number of people)
python3 - "$OUT" <<'PYEOF'
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from person_tracker_interfaces.msg import DetectedEntities

PEOPLE = {'cruzar': 1, 'duo_normal_mov': 2, 'duo_partial_oclusion': 2, 'duo_reid': 2,
          'duo_reid_cruzar': 2, 'one_normal_mov': 1, 'one_reid': 1, 'trio_normal_mov': 3,
          'trio_partial_oclusion': 3}
out = sys.argv[1]

print(f'\n{"bag":24} {"personas":>8} {"IDs":>5}')
for bag in sorted(os.listdir(out)):
    path = os.path.join(out, bag)
    if not os.path.exists(os.path.join(path, 'metadata.yaml')):
        continue
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/tracked_entities']))
    ids = set()
    while r.has_next():
        _, data, _ = r.read_next()
        for e in deserialize_message(data, DetectedEntities).entities:
            ids.add(e.id)
    print(f'{bag:24} {PEOPLE.get(bag, "?"):>8} {len(ids):>5}')
PYEOF
