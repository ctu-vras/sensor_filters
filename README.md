<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# sensor\_filters

This package is a collection of ROS 2 nodes and composable components that run a `filters::FilterChain` for message types
from the `sensor_msgs` package.
See the [filters](https://github.com/ros/filters) package documentation to learn more about the filter chain
infrastructure.

Attention: The *PCL filters* provided by package [pcl_ros](https://github.com/ros-perception/perception_pcl) are not
"compatible" with this package. They are instances of *PCL filters*, but written as ROS components/nodes, not as *ROS
filters* implementing the `filters::FilterBase<>` interface required by this package.

The task of each filter chain node in `sensor_filters` is very simple: load the filter chain, subscribe to the `input`
topic, and publish the filtered messages on the `output` topic.

Most behavior is configured via ROS 2 parameters declared on the node:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_queue_size` | int | 10 | Queue size used for the input subscription. |
| `output_queue_size` | int | 10 | Queue size used for the output publisher. |
| `subscription_type` | string | `unique_ptr` | How the input message is received. One of `unique_ptr`, `shared_ptr`, `reference`. Image and PointCloud2 chains require `shared_ptr`. |
| `publication_type` | string | `unique_ptr` | How the output message is published. One of `unique_ptr`, `shared_ptr`, `reference`. PointCloud2 chain does not support `unique_ptr`. |
| `is_lazy` | bool | `false` | If true, the chain only subscribes to the input topic when at least one subscriber is connected to the output topic (requires ROS 2 Iron+). |
| `content_filter_expression` | string | `""` | DDS content-filter expression for the input subscription (when supported by the RMW). |
| `content_filter_parameters` | string[] | `[]` | Parameters of the DDS content-filter expression. |
| `image_transport` | string | `raw` | Image chain only — input image transport. |
| `point_cloud_transport` | string | `raw` | PointCloud2 chain only — input point cloud transport. |

QoS settings of the input and output topic can be tuned via the standard
[`qos_overrides`](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-policy-overrides)
mechanism (see the example `filter.yaml` below).

## Provided executables

For each supported `sensor_msgs` type the package builds two executables and two composable components:
the plain node `<type>_filter_chain` and its lifecycle counterpart `<type>_filter_chain_lifecycle`.

| Message Type       | Node name and parameters namespace   | Component name                                      |
|--------------------|--------------------------------------|-----------------------------------------------------|
| BatteryState       | `battery_state_filter_chain`         | `sensor_filters::BatteryStateFilterChainNode`       |
| CameraInfo         | `camera_info_filter_chain`           | `sensor_filters::CameraInfoFilterChainNode`         |
| CompressedImage    | `compressed_image_filter_chain`      | `sensor_filters::CompressedImageFilterChainNode`    |
| FluidPressure      | `fluid_pressure_filter_chain`        | `sensor_filters::FluidPressureFilterChainNode`      |
| Illuminance        | `illuminance_filter_chain`           | `sensor_filters::IlluminanceFilterChainNode`        |
| Image              | `image_filter_chain`                 | `sensor_filters::ImageFilterChainNode`              |
| Imu                | `imu_filter_chain`                   | `sensor_filters::ImuFilterChainNode`                |
| JointState         | `joint_state_filter_chain`           | `sensor_filters::JointStateFilterChainNode`         |
| Joy                | `joy_filter_chain`                   | `sensor_filters::JoyFilterChainNode`                |
| JoyFeedbackArray   | `joy_feedback_array_filter_chain`    | `sensor_filters::JoyFeedbackArrayFilterChainNode`   |
| LaserScan          | `laser_scan_filter_chain`            | `sensor_filters::LaserScanFilterChainNode`          |
| MagneticField      | `magnetic_field_filter_chain`        | `sensor_filters::MagneticFieldFilterChainNode`      |
| MultiDOFJointState | `multi_dof_joint_state_filter_chain` | `sensor_filters::MultiDOFJointStateFilterChainNode` |
| MultiEchoLaserScan | `multi_echo_scan_filter_chain`       | `sensor_filters::MultiEchoLaserScanFilterChainNode` |
| NavSatFix          | `navsat_fix_filter_chain`            | `sensor_filters::NavSatFixFilterChainNode`          |
| PointCloud         | `pointcloud_filter_chain`            | `sensor_filters::PointCloudFilterChainNode`         |
| PointCloud2        | `pointcloud2_filter_chain`           | `sensor_filters::PointCloud2FilterChainNode`        |
| Range              | `range_filter_chain`                 | `sensor_filters::RangeFilterChainNode`              |
| RelativeHumidity   | `relative_humidity_filter_chain`     | `sensor_filters::RelativeHumidityFilterChainNode`   |
| Temperature        | `temperature_filter_chain`           | `sensor_filters::TemperatureFilterChainNode`        |
| TimeReference      | `time_reference_filter_chain`        | `sensor_filters::TimeReferenceFilterChainNode`      |

## Features

### Lifecycle support

For every chain executable, a managed (lifecycle) variant suffixed with `_lifecycle` is also available. Use the
[lifecycle CLI](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Managed-Nodes.html) to drive its states. The
lifecycle node only subscribes/publishes while in the `active` state; deactivating it stops the data flow without
destroying the chain.

Lifecycle component nodes are also available named like `sensor_filters::LifecycleBatteryStateFilterChainNode`.

### Transport support

Filters operating on `Image` and `PointCloud2` messages automatically use
[image_transport](https://github.com/ros-perception/image_common) and
[point_cloud_transport](https://github.com/ros-perception/point_cloud_transport) respectively to publish and subscribe
to topics. The input transport can be selected with the `image_transport` / `point_cloud_transport` parameter.

PointCloud2 transport uses `SensorDataQoS` by default for publications (in most cases). However, it is not a good
fit for filters which are usually treated as reliable chains. Therefore, the PointCloud2 filter defaults to reliable
publishers. If you need to use unreliable publishers by default, either use standard QoS overrides
or set parameter `default_best_effort_publisher` to true. For symmetry,
there is also parameter `default_best_effort_subscription` to control the subscriber side.

### Lazy subscription

Setting `is_lazy: true` causes the node to subscribe to the input topic only when there is at least one subscriber
connected to the output topic. As soon as the last output subscriber disconnects, the input subscription is dropped,
which is especially useful for filters that do expensive processing.

This requires ROS 2 Iron or newer (matched-event callbacks).

### Content filtering

`content_filter_expression` and `content_filter_parameters` are forwarded to
[rclcpp content filtering](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Topics.html#content-filtered-topics)
on the input subscription. This is supported by RMW implementations that implement DDS content-filtered topics
(e.g. Fast DDS, Connext). Other RMWs do the filtering on the subscription side which does not save traffic.

## Example usage

`filter.yaml`

```YAML
/**:
  ros__parameters:
    output_queue_size: 100
    publication_type: unique_ptr
    is_lazy: false
    laser_scan_filter_chain:
      filter1:
        # Warning: using laser_filters like this might fail if they need to wait for some TFs; intensity filter does not.
        name: intensity
        type: laser_filters/LaserScanIntensityFilter
        params:
          lower_threshold: 8000.0
          upper_threshold: 100000.0
          invert: false
          filter_override_range: true
          filter_override_intensity: false
      filter2:
        # This filter will subtract 25 ms from each scan timestamp, e.g. to account for transport delay.
        name: delay
        type: sensor_filters/ChangeHeader_LaserScan
        params:
          stamp_relative: -0.025
    qos_overrides:
      /base_scan:
        subscription:
          reliability: best_effort
```

(ROS 2 XML launch) `filter.launch`

```XML
<launch>
    <node pkg="sensor_filters" exec="laser_scan_filter_chain" name="laser_filter" output="screen">
        <remap from="input" to="base_scan" />
        <remap from="output" to="base_scan_filtered" />
        <param from="$(dirname)/filter.yaml" allow_substs="true" />
    </node>
</launch>
```

More ready-to-run examples (LaserScan, Image and PointCloud2, including optional sample-data publishers and lifecycle variants) are in the [`examples/`](https://github.com/ctu-vras/sensor_filters/tree/ros2/examples) directory.

## Provided filters

This package provides a high-level filter that can be used with any `sensor_msgs` type that contains a header.

### `sensor_filters/ChangeHeader_<MESSAGE_TYPE>`

Allows changing the contents of the header of sensor messages. Use it for example to correct the `frame_id` of a sensor,
or to adjust the timestamp of its messages.

The filter is registered for every `sensor_msgs` type that has a `header` field (see the list above). 

Note: ROS 2 messages no longer carry a `seq` field in their headers, so the ROS 1 `seq` and `seq_relative` parameters
have been removed.

#### Parameters

- `frame_id_prefix` (string): Add this prefix to `header.frame_id`.
- `frame_id_suffix` (string): Add this suffix to `header.frame_id`.
- `frame_id` (string): Replace `header.frame_id` with this value (prefix and suffix are then ignored).
- `stamp` (double): Set `header.stamp` to this absolute value (in seconds since epoch).
- `stamp_relative` (double): Add this value (in seconds) to `header.stamp`.

## Where to get other filters

If you are looking for other implementations of sensor filters, choices are scarce. Notable exceptions are:

### `robot_body_filter`

[robot_body_filter](https://github.com/peci1/robot_body_filter) is a versatile tool for removing the body parts of a
robot from laser scans and point clouds exactly according to the robot's URDF model.

### `point_cloud2_filters`

[point_cloud2_filters](https://github.com/torydebra/point_cloud2_filters) provides an implementation similar to
`pcl::PassThrough` or `pcl::CropBox` to cut `PointCloud2` messages.

### `laser_filters` compatibility

The `LaserScan` filter chain node is compatible with `laser_filters` from the
[laser_filters](https://github.com/ros-perception/laser_filters) package (it can load the same filters using the
same config). It does not, however, use a TF message filter, so each filter has to wait for the required TFs itself.

## Why ROS filters?

The simple answer is — performance. ROS filters are the most efficient way to run a chain of processors on sensor
data: passing the message from one filter to another is a plain C++ call. Compared to a chain of separate components
communicating via intra-process publish/subscribe, you avoid the allocation and bookkeeping of intra-process
shared-ptrs; compared to a chain of separate nodes communicating over the network stack, you also avoid
(de)serialization and the network round trip.

## Extensibility

You can build your own node on top of any chain by inheriting from `sensor_filters::FilterChainNode<T>` or
`sensor_filters::LifecycleFilterChainNode<T>`. The Image and PointCloud2 chains additionally provide
`ImageFilterChainBase` and `PointCloud2FilterChainBase`, which plug image/point cloud transport into the chain.

See the [examples folder](https://github.com/ctu-vras/sensor_filters/tree/ros2/examples) for end-to-end examples.

### Custom plain node

```c++
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_filters/FilterChainNode.hpp>

class MyNode : public sensor_filters::FilterChainNode<sensor_msgs::msg::LaserScan> {
public:
  explicit MyNode(const rclcpp::NodeOptions& options) : FilterChainNode("my_filter_chain", options) {}
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MyNode)
```

### Custom lifecycle node

```c++
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_filters/FilterChainNode.hpp>

class MyLifecycleNode : public sensor_filters::LifecycleFilterChainNode<sensor_msgs::msg::LaserScan> {
public:
  explicit MyLifecycleNode(const rclcpp::NodeOptions& options) : LifecycleFilterChainNode("my_filter_chain", options) {}
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MyLifecycleNode)
```

### Writing a universal filter

To register a single templated `filters::FilterBase<T>` implementation for every type, include `<sensor_filters/RegisterFilter.hpp>` and use the `REGISTER_UNIVERSAL_SENSOR_MSGS_FILTER` macro (and/or `REGISTER_NO_HEADER_SENSOR_MSGS_FILTER` to handle messages without a header). On the CMake side, call `register_universal_filter_description_file()` to generate and install the corresponding `pluginlib` XML description file. See the `ChangeHeader` filter for a complete example.

## Migration from ROS 1

Version 1.x of this package targeted ROS 1 and provided both nodes and nodelets.
With the ROS 2 rewrite, the public API, executable names and parameter layout changed considerably.
The list below summarizes the most important differences when porting an existing ROS 1 configuration:

- **Nodelets removed.** ROS 2 has no nodelets; the package now ships _composable components_ registered via `rclcpp_components`. Each filter chain comes both as a standalone executable and as a component that can be loaded into a `component_container`.
- **Lifecycle nodes added.** Every executable also has a `_lifecycle` variant based on `rclcpp_lifecycle::LifecycleNode`
- **header.seq is missing in ROS 2.** `ChangeHeader` filter's parameters `seq` and `seq_relative` have been removed.
- **Executable names.** Most chain executables were renamed.
- **More chains are supported.** `battery_state_*`, `camera_info_*`, `fluid_pressure_*`, `illuminance_*`, `joint_state_*`, `joy_feedback_array_*`, `multi_dof_joint_state_*`, `time_reference_*`.
- **Different way of configuring filter chains in ROS 2.** See the example below to see how the syntax changed (you have to use consecutive names `filter1`, `filter2`, etc.).
- **LaserScan chain parameter namespace.** LaserScan filter chain is read from `laser_scan_filter_chain` instead of the ROS 1 `scan_filter_chain`. Adjust your YAML accordingly: 

```YAML
  # ROS 1
  scan_filter_chain:
    - name: ...
      type: ...
```

```YAML

  # ROS 2
  /**:
    ros__parameters:
      laser_scan_filter_chain:
        filter1:
          name: ...
          type: ...
```

- **Universal filter naming.** Filter class names use an underscore between the filter name and the message type:
    - `sensor_filters/ChangeHeader/LaserScan` → `sensor_filters/ChangeHeader_LaserScan`
- **Topic remaps drop the `~` prefix.** ROS 2 launch uses topic names without the leading tilde, e.g. `<remap from="input" to="base_scan" />`.
- **Parameter remaps.** `~input_queue_size`/`~output_queue_size` are now ordinary ROS 2 parameters `input_queue_size`/`output_queue_size` and must be set under the `ros__parameters` key of a YAML config (or via `--ros-args -p ...`).
- **Includes.** Public headers moved from `.h` to `.hpp`, and `FilterChainNodelet.h` is gone — extend `sensor_filters::FilterChainNode<T>` or `sensor_filters::LifecycleFilterChainNode<T>` (optionally with the `ImageFilterChainBase` / `PointCloud2FilterChainBase` base) instead.