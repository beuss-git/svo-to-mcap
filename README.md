This project allows you to convert [stereolabs/zed svo files](https://www.stereolabs.com/docs/video/recording) files [mcap](https://mcap.dev/) files for offline foxglove playback.

You can also easily merge a ROS 2 mcap capture via the [mcap CLI tool](https://mcap.dev/guides/cli) using `mcap merge ...`.

This tool currently only supports `sensor_msgs/msg/Image`, but support for others can easily be added.
Time synchronization is based on setting the ROS 2 header's timestamp field to [TIME_REFERENCE::IMAGE](https://www.stereolabs.com/docs/api/group__Video__group.html#ga9401e0c9b9fec46d2eb300ffd2fc72c9).
