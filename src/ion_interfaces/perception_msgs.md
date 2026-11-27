# Perception Interface Proposal

Lidar cone detection (`ion_perception`, node `lidar_listener`). Camera detection is not in yet.

## Inputs
| Topic                        | Type                          | Used for                                      |
|------------------------------|-------------------------------|-----------------------------------------------|
| /lidar/points                | sensor_msgs/msg/PointCloud2   | Lidar scans (x forward, y left, z up), 10 Hz  |

## Outputs
| Topic                        | Type                          | Used for                                      |
|------------------------------|-------------------------------|-----------------------------------------------|
| /perception/cones            | ion_interfaces/msg/ConeArray  | Detected cones, one message per scan          |
| /lidar/cone_markers          | visualization_msgs/msg/MarkerArray | Cones in RViz (debugging only)           |
| /lidar/above                 | sensor_msgs/msg/PointCloud2   | Points left after ground removal (debugging)  |

## Output format: /perception/cones

The message types (`msg/ConeArray.msg`, `msg/Cone.msg`):

```
ConeArray:  std_msgs/Header header
            Cone[] cones

Cone:       geometry_msgs/Point position     # m
            string color
            float32 confidence               # 0..1
```

- One message for every lidar scan, also when no cones are seen (then `cones: []`), so the next block can
  tell "no cones" apart from "perception has stopped".
- `header`: copied from the scan, so `stamp` is the scan time and `frame_id` is `lidar`.
- `position`: the middle of the cone's points, in the lidar frame: x forward, y left, z up, from the lidar.
- `color`: always `"unknown"` for now: the lidar can't see colour. Colour comes with the camera.
- `confidence`: always `1.0` for now (a placeholder until there is a real score).
- The cones are in no particular order.

A real message (`ros2 topic echo /perception/cones`, a simulated Livox Mid-360S scan, cut to 3 of 8 cones):

```yaml
header:
  stamp:
    sec: 1795784830
    nanosec: 909895769
  frame_id: lidar
cones:
- position:
    x: 2.7422499656677246
    y: 1.7127550840377808
    z: -0.0198702160269022
  color: unknown
  confidence: 1.0
- position:
    x: 3.023477554321289
    y: -1.7167789936065674
    z: -0.002238987246528268
  color: unknown
  confidence: 1.0
- position:
    x: 7.225716590881348
    y: -1.109694242477417
    z: 0.02776549756526947
  color: unknown
  confidence: 1.0
```

A scan with no cones:

```yaml
header:
  stamp:
    sec: 1795784834
    nanosec: 699648206
  frame_id: lidar
cones: []
```

## Open questions
- **Frame**: are cones in the lidar frame OK for state estimation (convert with TF), or should perception publish
  them in the car frame (`base_link`)?
- **Colour type**: `string` (current `Cone.msg`) or `uint8` with constants (`BLUE`, `YELLOW`, `ORANGE_BIG`,
  `ORANGE_SMALL`, `UNKNOWN`)? A string is readable in `ros2 topic echo`; constants catch typos like `"Blue"` vs `"blue"`.
- **Lidar topic and mount**: `/lidar/points` is what our own simulator publishes; the real driver's topic and the
  mount height (assumed 0.18 m, used for ground removal) are still to be decided.
