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
## Result
![Screenshot from 2021-02-21 19-42-46](https://user-images.githubusercontent.com/52307432/108654719-9d0f1200-750c-11eb-9449-3e04c32b6784.png)  
https://user-images.githubusercontent.com/52307432/108654859-e3fd0780-750c-11eb-9b66-108e4c4677b3.mp4

