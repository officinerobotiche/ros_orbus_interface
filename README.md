# ![Officine Robotiche][Logo] - ros_orbus_interface

With this node you can control with ROS all boards with orbus protocoll, in particular:
- [**µNAV**](http://unav.officinerobotiche.it) The motion control board for two DC brushed motors.
- [**Navigation Board**](http://raffaello.officinerobotiche.it/boards/old-boards/navigation-board/) A board to read all data from infrared sensor

# Release
- [**Download last stable release**](https://github.com/officinerobotiche/ros_serial_bridge/releases)
- [wiki] with all detailed information about firmware

# Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/officinerobotiche/ros_serial_bridge
cd ~/catkin_ws
catkin_make
```
Build your ros package:
```bash
catkin_create_pkg myrobot_tutorial orbus_interface diff_drive_controller joint_state_controller
cd myrobot_tutorial
mkdir launch
cd launch
```
And create your personal launch script:
```xml
<launch>
    <!-- Your script with your URDF file -->
    <param name="robot_description" file="$(find myrobot_tutorial)/urdf/myrobot.urdf"/>

    <!-- ORBus harware driver -->
    <node pkg="orbus_interface" type="hardware_unav" name="unav" output="screen">
        <rosparam subst_value="true">
            serial_port: /dev/ttyUSB0
            serial_rate: 115000

            control_frequency: 10.0
            diagnostic_frequency: 1.0
        </rosparam>
    </node>

    <!-- Differential controller parameters and basic localization -->
    <rosparam command="load" file="$(find myrobot_tutorial)/config/control.yaml" />
    
    <node name="base_controller_spawner" pkg="controller_manager" type="spawner" 
            args="unav_joint_publisher unav_velocity_controller --shutdown-timeout 3"/>
    
    <!-- Diagnostic Aggregator -->
    <!--<node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostic_aggregator">
        <rosparam command="load" file="$(find husky_base)/config/diagnostics.yaml"/>
    </node>-->
</launch>
```
and save `driver.launch`
# Run
Now you can launch you personal script with
```bash
roslaunch myrobot_tutorial driver.launch
```

# API
## Published Topics

## Subscribed Topics

[wiki]:http://wiki.officinerobotiche.it/
[Officine Robotiche]:http://www.officinerobotiche.it/
[Logo]:http://2014.officinerobotiche.it/wp-content/uploads/sites/4/2014/09/ORlogoSimpleSmall.png
