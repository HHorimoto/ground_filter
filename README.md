# ground_filter

![melodic workflow](https://github.com/HHorimoto/ground_filter/actions/workflows/melodic.yml/badge.svg)
![noetic workflow](https://github.com/HHorimoto/ground_filter/actions/workflows/noetic.yml/badge.svg)

**This package filters ground by PointCloud2**

## Requirement
+ ROS Melodic (on Ubuntu 18.04 LTS, build and run test on Github Actions)
+ ROS Noetic (on Ubuntu 20.04 LTS, build and run test on Github Actions)

## Test
This package provides test bash for use with `Github Actions`.
This test confirms if `/points_lanes` and `/points_ground` are published or not by rosbag.
You can also do this test on your computer by following this command.
I belive that you can see `Success` not `Fail`.

```shell
$ roscd ground_filter
$ bash -xv test/test_melodic.bash # or test_noetic.bash
# You can see result.
$ killall -9 rosmaster # kill roscore
```

## License

Distributed under the BSD-3-Clause License. See `LICENSE` for more information.