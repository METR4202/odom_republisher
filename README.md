# odom_republisher

Package for METR4202 Prac 2 odometry republishing exercise.

### Usage

    cd ~/path/to/ws/src
    git clone https://github.com/METR4202/odom_republisher.git
    cd ..
    colcon build --symlink-install
    ros2 run odom_republisher odometry_republisher

### Tips

If you receive a runtime error regarding the use of `float` and `numpy`, make sure to upgrade to the latest version of the `transforms3d` library:

    pip install transforms3d==0.4.2