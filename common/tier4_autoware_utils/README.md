# tier4_autoware_utils

## Purpose

This package contains many common functions used by other packages, so please refer to them as needed.

A nearest index of the ego pose or an object pose is assumed to be the first nearest index within distance and angle thresholds.

This Assumption is effective to crossing routes as follows.

Defined `calcSignedArcLength` functions will be introduced here.

- from index to index
```
template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx);
```

- from point with index to point with index
```
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_idx);
```

- from point with index to index
   - e.g.
       - Source is the stop line, whose nearest index is calculated with lane id beforehand.
       - Destination point is the zero velocity index calculated with another function beforehand.
```
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t src_idx, const size_t dst_idx);
```

- from pose to index
   - e.g.
       - Source is the ego pose, whose nearest index is assumed to be the first nearest index within distance and angle thresholds.
       - Destination is the zero velocity index.
```
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Pose & src_pose, const size_t dst_idx);
```

- from pose to pose
   - e.g.
       - Source is the ego pose, whose nearest index is assumed to be the first nearest index within distance and angle thresholds.
       - Destination is an object pose, whose nearest index is assumed to be the first nearest index within distance and angle thresholds.
```
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Pose & src_pose, const geometry_msgs::msg::Pose & dst_pose);
```
