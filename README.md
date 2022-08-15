# sensor\_filters

This package is a collection of nodes and nodelets that service a `filters::FilterChain` for message types from `sensor_msgs` package.
See https://wiki.ros.org/filters to read more about the `filters` package.

Attention: The *PCL filters* provided by package [pcl_ros](http://wiki.ros.org/pcl_ros/Tutorials/filters) are not "compatible" with this package. They are instances of *PCL filters*, but written as ROS nodelets, not as *ROS filters* implementing the `filters::FilterBase<>` interface required by this package. Running pcl_ros filter nodelets does not require any additional setup except a running nodelet manager.

Each node/nodelet's task is very simple: load the filter chain, subscribe `~input` topic, and publish the filtered messages to `~output` topic.

The size of message queues is configured via `~input_queue_size` and `~output_queue_size`, both defaulting to 10 messages.

The namespaces from which the filters load are the following:

| Message Type | Namespace | Nodelet Name |
|--------------|-----------|--------------|
| CompressedImage | `~image_filter_chain` | `sensor_filters/compressedimage_filter_chain` |
| Image | `~image_filter_chain` | `sensor_filters/image_filter_chain` |
| Imu | `~imu_filter_chain` | `sensor_filters/imu_filter_chain` |
| Joy | `~joy_filter_chain` | `sensor_filters/joy_filter_chain` |
| LaserScan | `~scan_filter_chain` | `sensor_filters/laserscan_filter_chain` |
| MagneticField | `~magnetic_field_filter_chain` | `sensor_filters/magneticfield_filter_chain` |
| MultiEchoLaserScan | `~scan_filter_chain` | `sensor_filters/multiecholaserscan_filter_chain` |
| NavSatFix | `~nav_sat_fix_filter_chain` | `sensor_filters/navsatfix_filter_chain` |
| PointCloud | `~cloud_filter_chain` | `sensor_filters/pointcloud_filter_chain` |
| PointCloud2 | `~cloud_filter_chain` | `sensor_filters/pointcloud2_filter_chain` |
| Range | `~range_filter_chain` | `sensor_filters/range_filter_chain` |
| RelativeHumidity | `~humidity_filter_chain` | `sensor_filters/relativehumidity_filter_chain` |
| Temperature | `~temperature_filter_chain` | `sensor_filters/temperature_filter_chain` |

## Provided filters

This package also provides a few high-level filters.

### sensor_filters/ChangeHeader/$MESSAGE_TYPE

This filter allows changing the contents of the header of sensor messages. You can use it for example to correct the `frame_id` of a sensor, or the adjust the timestamp of its messages.

#### Parameters

- `frame_id_prefix` (string): Add this prefix to `header.frame_id`
- `frame_id_suffix` (string): Add this suffix to `header.frame_id`
- `frame_id` (string): Replace `header.frame_id`with this value (prefix and suffix are then ignored)
- `seq` (uint): Replace `header.seq` with this value
- `seq_relative` (uint): Add this value to `header.seq` (watch out for unsigned int underflow)
- `stamp` (double): Set `header.stamp` to this value (double value is converted to `ros::Time`)
- `stamp_relative` (double): Add this value to `header.stamp` (double value is converted to `ros::Duration`)

## Where to get other filters

If you are looking for other implementations of sensor filters, you will probably be disappointed, as they are really scarce. I don't know why it is the case, because filter chains are the most efficient data processing path ROS offers, yet most people chose to write nodelets instead.

There is, however, one noteworthy exception:

### `robot_body_filter`

[robot_body_filter](http://wiki.ros.org/robot_body_filter) is a versatile tool for removing the body parts of a robot from laser scans and point clouds. No more box approximations of your robots! Represent them exactly as they are!

### `laser_filters` compatibility

The `LaserScan` filter chain node is compatible with the `scan_to_scan_node` from `laser_filters` package (it can load the same filters using the same config).
It does not, however, use TF message filter, so each filter has to wait for the required TFs itself.

## Extensibility

Both the nodes and nodelets offer common code that can be shared so that you can build extensions of the simple filter chain handlers provided by this package.

See the [examples folder](https://github.com/ctu-vras/sensor_filters/blob/master/examples) for more verbose examples.

### Nodes

```C++
#include <sensor_msgs/PointCloud2.h>
#include <sensor_filters/FilterChainNode.h>

class MyClass : public sensor_filters::FilterChainNode<sensor_msgs::PointCloud2>
{
  // custom class code
};
```

### Nodelets

```C++
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_filters/FilterChainNodelet.h>

class MyNodelet : public sensor_filters::FilterChainNodelet<sensor_msgs::PointCloud2>
{
  protected: void onInit() override
  {
    sensor_filters::FilterChainNodelet<sensor_msgs::PointCloud2>::onInit();
    // custom nodelet code
  }
};

PLUGINLIB_EXPORT_CLASS(MyNodelet, nodelet::Nodelet)
```

## Example

`filter.yaml`

```YAML
output_queue_size: 100
scan_filter_chain:
  # Warning: using laser_filters like this might fail if they need to wait for some TFs; intensity filter does not.
  - name: intensity
    type: laser_filters/LaserScanIntensityFilter
    params:
      lower_threshold: 8000
      upper_threshold: 100000
      invert: false
      filter_override_range: true
      filter_override_intensity: false
  # This filter will subtract 25 ms from each scan timestamp, e.g. to account for transport delay.
  - name: delay
    type: sensor_filters/ChangeHeader/LaserScan
    params:
      stamp_relative: -0.025
```

`filter.launch`

```XML
<launch>
    <node pkg="sensor_filters" type="laserscan_filter_chain" output="screen" name="laser_filter">
        <remap from="~input" to="base_scan" />
        <remap from="~output" to="base_scan_filtered" />
        <rosparam command="load" file="$(dirname)/filter.yaml" />
    </node>
</launch>
```