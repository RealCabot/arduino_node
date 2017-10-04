## Arduino Firmware

For the time being, it only have the functionality of reading encoders.

## How to build and upload it

Note: you **CANNOT** build it using Arduino IDE. Instead, use `catkin` to manage the firmware just as other nodes.

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

### Compile and upload

- Compile the firmware: `catkin_make arduino_node_firmware_arduinoEncoder`
- Upload the firmware: `catkin_make arduino_node_firmware_arduinoEncoder-upload`

## What to expect

- Run roscore: `roscore`
- Run this to forware Arduino message: `rosrun rosserial_python serial_node.py /dev/ttyACM0`
    - You may need to change the serial port name (The name can be seen in Arduino IDE)
- Move the wheel and see: `rostopic echo encoder`

## F & Q

- It builds successfully but won't upload
    - First check that the port name is `/dev/ttyACM0`, if not, edit the `PORT` part in `firmware/CMakeList.txt`.
    - Unplug the USB, plug it back and try again.