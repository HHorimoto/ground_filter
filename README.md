# ground_filter

![melodic workflow](https://github.com/HHorimoto/ground_filter/actions/workflows/melodic.yml/badge.svg)
![noetic workflow](https://github.com/HHorimoto/ground_filter/actions/workflows/noetic.yml/badge.svg)

**This node removes the ground points from the input pointcloud by RANSAC Algorithm**

The demo video can be found here -> https://youtu.be/eCVASqiWuVc

## Requirement
+ ROS Melodic (on Ubuntu 18.04 LTS, build and run test on [Github Actions](./.github/workflows/melodic.yml))
+ ROS Noetic (on Ubuntu 20.04 LTS, build and run test on [Github Actions](./.github/workflows/noetic.yml))

## Set Up
Download `ground_filter` package.

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/HHorimoto/ground_filter.git
$ cd ~/catkin_ws && catkin_make
```

## How to Use
Launch `ground_filter.launch`

```shell
$ roslaunch ground_filter ground_filter.launch
```


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