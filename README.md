[![CircleCI](https://circleci.com/gh/ros-drivers/velodyne.svg?style=svg)](https://circleci.com/gh/ros-drivers/velodyne)

Overview
========

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high definition 3D LIDARs`<sup>3</sup>.

**Warning**:

  The master branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.
  To check out the source for the most recent release, check out the
  tag `velodyne-<version>` with the highest version number.

The current ``master`` branch works with ROS Indigo and Kinetic.
CI builds are currently run for Lunar and Melodic but extensive
testing has not been completed in those environments.

- <sup>1</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>2</sup>ROS: http://www.ros.org
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx

For this `README.md` file, please note that Speed bump code is present in the `/velodyne_pointcloud/src/Speed_bump_code.cpp`.

---

To use the Speed bump code, follow these steps:

1. **Create a Catkin workspace**:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    ```

2. **Clone this GitHub repository** to the workspace:
    ```bash
    git clone https://github.com/ros-drivers/velodyne.git
    ```

3. **Build the files**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

4. **Source the setup.bash file** to link the executable to the terminal:
    ```bash
    source devel/setup.bash
    ```

5. **Run Speed_bump_code.cpp**:
    - First, run a bag file containing the lidar data with speed bump points included.
    - Then, use `rosrun` to execute the Speed bump code:
        ```bash
        rosrun velodyne_pointcloud Speed_bump_code
        ```

