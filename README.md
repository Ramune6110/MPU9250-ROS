# MPU9250-ROS

## MPU9250 with Arduino
[Use the following libraries](https://github.com/asukiaaa/MPU9250_asukiaaa)

## Execution method with imu_tools
### STEP1
```
roscore
```
### STEP2
```
sudo chmod 666 /dev/ttyACM0
```
Write mpu9250.ino to the Arduino
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

## Movie
[rviz movie](https://user-images.githubusercontent.com/52307432/108654859-e3fd0780-750c-11eb-9b66-108e4c4677b3.mp4)

## Execution method with madgwick filter
### STEP1
```
roscore
```
### STEP2
```
sudo chmod 666 /dev/ttyACM0
```
Write mpu9250_with_madgwick.ino to the Arduino
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
### STEP3
```
source devel/setup.bash
```
```
roslaunch MPU9250 imu_with_madgwick.launch
```
## Result
![Screenshot from 2021-02-21 22-55-20](https://user-images.githubusercontent.com/52307432/108673693-714d5580-7527-11eb-9c85-4e4ca513c116.png)


## Movie
[rviz movie](https://user-images.githubusercontent.com/52307432/108673874-c38e7680-7527-11eb-8763-f9938b3f7b3e.mp4)
