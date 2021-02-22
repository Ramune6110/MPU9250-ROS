# MPU9250-ROS

## MPU9250 with Arduino
[Use the following libraries](https://github.com/asukiaaa/MPU9250_asukiaaa)

## Execution method
### STEP1
```
roscore
```
### STEP2
```
sudo chmod 666 /dev/ttyACM0
```
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
### STEP3
```
source devel/setup.bash
```
```
roslaunch MPU9250 imu.launch
```

