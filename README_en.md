# person-tracker
This package tracks a person when they leave the camera's field of view and recognizes them when they come back.

*Versión en español: [README.md](README.md)*

Project developed in a computer with a *(GPU) NVIDIA RTX 4060* graphic processing unit, with the `oak_d_camera` by *Luxonis* and the *RPLIDAR* LiDAR.

## How it works
The system combines the camera and the LiDAR through several nodes:
- **DetectionTo3DfromDepthNode:** it receives the YOLO detections, lifts them to 3D using the depth image and transforms them to `odom`. It publishes on `/detections_3d_odom`.
- **ObjectProcessingNode:** it segments the LiDAR scan into objects and publishes their centers in `odom` on `/lidar_objects`.
- **PersonTrackerNode:** it fuses both sources. Only the camera creates entities (people) and the LiDAR updates them, also outside the camera's field of view, predicting their position from their velocity. It publishes on `/tracked_entities`.
- **TrackerDebugNode:** it publishes the RViz markers on `/debug/tracked_entities_markers`.

**COMMANDS**
Activate `debug` to visualize the entities in RViz.
```bash
ros2 launch person_tracker person_tracker.launch.py debug:=true
```

## Branches
- **cambios-viejos:** backup of my original approach (commit `64d82c5`), where the LiDAR created one entity for every detected object.
- **main:** contains my change sketch (`05f2347`, which did not compile) and the corrected version I explain below.

## Troubleshooting
### Design problem
In my original approach (branch `cambios-viejos`) the nodes were connected like this:
- **ObjectProcessingNode:** it segmented the LiDAR scan and published the object centers on `/lidar_objects`.
- **LidarTrackerNode:** it tracked every LiDAR object. It created a new entity for every unassociated center, associated it in the next scans with the closest point (within 0.6 m), predicted its position from its velocity and deleted it after `max_misses` scans without seeing it. It published the entities on `/tracked_entities`.
- **DetectionTo3DfromDepthNode:** it lifted the YOLO detections to 3D in `odom` and published them on `/detections_3d_odom`.
- **PersonTrackerNode (`sensor_fusion`):** the idea was to synchronize `/tracked_entities` and `/detections_3d_odom` with `message_filters` (`ApproximateTime`) and decide which LiDAR entity was each person detected by YOLO. I never got to implement this node.

When testing it I realized this approach was not viable. The LiDAR also detects walls, chair legs, columns and separate legs, and in the bags it gave between 20 and 30 objects per scan. Since the LiDAR created an entity for each of them, there were so many entities being tracked at the same time, many of them very close to each other, that when associating by proximity the IDs kept switching all the time, and ghost points also appeared. With LiDAR IDs changing constantly, the fusion node could not know which entity was each person, so it made no sense to match them afterwards.

To solve it I changed the architecture: now only the camera creates entities, since only YOLO knows that something is a person, and the LiDAR only updates entities that already exist. LiDAR objects not associated with any entity are ignored.

### Other changes
Besides the architecture change, I made the following changes with respect to my original approach (`64d82c5`). I justified all of them by measuring with the bags in `bags/corrected_bags`.

#### Tracker
- **Global greedy association:** before, each entity, in order, took the closest free point, so the first one in the list could "steal" another entity's point. I created the `associate_closest` function, which computes all entity–point pairs within the margin, sorts them from closest to farthest and assigns them without repetition. It is used by the LiDAR (0.6 m margin) and by the camera (1.0 m). With several people close together the IDs got swapped, and with two people side by side the second one was never created.
- **The camera keeps alive an entity the LiDAR has lost:** if the camera sees a person whose entity has `misses > 0`, it sets the camera position and `misses = 0`. Otherwise, someone seen by the camera but not by the LiDAR (far away, next to another object or occluded by another person) was deleted after `max_misses` scans.

  To check whether this rule was needed, I ran the 9 bags with and without it (one run per version). Without the rule there were 68 IDs in total, against 40 with it, and no bag improved. The biggest differences appeared with occlusions, because the LiDAR only sees a plane at leg height and gets occluded easily, while the camera sees the whole person:
  - **duo_reid:** 14 IDs without the rule against 4 with it.
  - **trio_partial_oclusion:** 18 IDs without the rule against 6 with it.
  - **Other bags:** differences of 0 to 2 IDs.
- **Velocity only between LiDAR positions:** I added the `position_from_lidar` field to compute the velocity only between two LiDAR positions. Subtracting a camera position from a LiDAR position produces false spikes, because each sensor has its own error.
- **Velocity filter:** I added an exponential moving average `v = 0.3·v_measured + 0.7·v_previous`, which trusts the new velocity 30 % and the previous one 70 %. I did it this way because the velocity computed directly between two LiDAR positions sometimes had very large spikes, for example because of the flickering of some centroids, and with them the prediction went far from the person and the ID changed.
- **Prediction during occlusions:** before, I only predicted if the entity had been seen in the previous scan (`misses == 0`) and for less than 0.4 s, and when it was lost the velocity was set to 0. I had done it this way to avoid velocity jumps, but I solved them in another way with the exponential moving average (see the velocity filter). Besides, conceptually this made no sense for my approach, since the idea is to predict precisely when the person is not seen. Now I predict as long as the entity exists (`dt > 0`), the limit is set by `max_misses`, and the velocity is no longer set to 0.

#### Other nodes
- **Invalid depths:** in `DetectionTo3DfromDepthNode`, besides `NaN`, I now discard depths of 0 (pixel without data) and above 8 m. They produced detections on top of the robot or 30 m away that created ghosts, especially with the person at the image border.

#### Parameters
- **max_dist from 0.50 to 0.30:** in `config/params.yaml`, it is the maximum distance between two consecutive scan points for them to belong to the same object. With 0.5, a person in front of a wall merged with it into a single object that was too wide, which was discarded for exceeding `max_obj_size`.
- **max_misses from 5 to 8:** in `config/params.yaml`, it is the number of consecutive scans without seeing an entity after which it is deleted. With 5 entities were deleted too early when the LiDAR lost the person for a moment, and they got a new ID when they came back. I tried 10 and there was less fragmentation, but sometimes the entity survived the loss of the person and got stuck to a nearby static object that the LiDAR sees in every scan, staying there forever. I left it at 8 as a compromise between both cases.

#### Reorganization
I moved all the tracker code to `PersonTrackerNode` (node `person_tracker_node`) and removed `LidarTrackerNode` and `lidar_tracker_main`. In `params.yaml` I renamed the `detections_tracking_node` section to `person_tracker_node`.

### Observations
- **The camera and the LiDAR arrive at different moments:** that is why I do not synchronize both sensors in the tracker, since waiting to have one message from each would leave the positions outdated. Each sensor updates the entities as soon as its message arrives. My hypothesis is that the camera detections arrive later because of the YOLO and depth processing.
- **Variation between runs:** results change from one run to another probably because YOLO's processing time is not always the same. To compare versions, each bag must be run several times.

## Methodology
I have developed several scripts in [scripts/eval](/scripts/eval) to evaluate the tracker with the bags:
- **[run_bags.sh](/scripts/eval/run_bags.sh):** it runs YOLO and the tracker on each bag, records the topics and at the end shows a table with the total number of IDs of each bag against the real number of people. It uses `ROS_DOMAIN_ID=77` to avoid interfering with other processes.
- **[ids.py](/scripts/eval/ids.py):** it shows when and where each ID is born and dies, and in what percentage of scans the LiDAR saw it.

**COMMANDS**
```bash
source ~/TFG_ws/install/setup.bash
src/person_tracker/scripts/eval/run_bags.sh <results_folder> [bags]
```

## Known limitations
- **Lost by both sensors:** if the camera and the LiDAR lose the person, they get a new ID when they come back.
- **Crossings outside the camera:** if the tracked person passes next to another standing person, or a static object, outside the camera's field of view, the ID can jump to it (bag `cruzar`).
- **Ghosts:** a one-off depth error can create an ID that disappears right away.
