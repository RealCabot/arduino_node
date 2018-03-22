## Arduino Firmware

### Structure

All the sensors implement the [abstract class](http://en.cppreference.com/w/cpp/language/abstract_class) `SensorReader`, which offers a uniform interface for all the sensors.

The Timer functionality is implement using polling because of the stupid incapability between `Timer` library and `ros_serial`. In the future consider using `Timer1` library.

### How to build and upload the firmware

Note: you **CANNOT** build it using Arduino IDE. Instead, use `catkin` to manage the firmware just as other nodes. Run the following command to compile and upload the firmware.

```
catkin_make arduino_node_firmware_oneForAll-upload
```

### How this interacts with ROS

- Run this first  to forward Arduino message: `roslaunch arduino_node ros_serial.launch`
    - You may need to change the serial port name (The name can be seen in Arduino IDE)
- Published:
    - `/encoder`: the speed information of motor, using `arduino_msg::Motor`
    - `/imu`: absolute pose information, using `geometry_msg::Vector3Stamped`
- Subscribed:
    - `/motorSpeed`: set the motor desired speed.
        - You can publish in terminal like this: `rostopic pub /motorSpeed arduino_msg/Motor '{left_speed: 0.4, right_speed: 0}' -1`
    - `/tunePID_L` and `/tunePID_R`: set `Kp`, `Ki` and `Kd`
        - You can publish in terminal like this: `rostopic pub /tunePID_L geometry_msgs/Vector3 '{x: 10, y: 5, z: 10}' -1`

### Install dependencies

```
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```
or
```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```

Then pull the git submodules (libraries):
```
git submodule update --init --recursive
```

Then install the Arduino IDE for Linux

## FAQ

- It builds successfully but won't upload
    - First check that the port name is `/dev/ttyACM0`, if not, edit the `PORT` part in `firmware/CMakeList.txt` AND change the `port` value in `launch/ros_serial.launch` to `/dev/ttyACM0`
    - Unplug the USB, plug it back and try again.
    