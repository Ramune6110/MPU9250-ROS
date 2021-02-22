#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mpu;

/***********************************************************************
 * Global variables
 **********************************************************************/
float aX = 0.0f;
float aY = 0.0f;
float aZ = 0.0f;
float gX = 0.0f;
float gY = 0.f;
float gZ = 0.0f;
float mX = 0.0f;
float mY = 0.0f;
float mZ = 0.0f;
float aSqrt      = 0.0f;
float mDirection = 0.0f;
float gXOffset   = 0.3f;
float gYOffset   = 1.3f;
float gZOffset   = -0.9f;

/**********************************************************************
 * ROS Parameter
 **********************************************************************/
ros::NodeHandle nh;

sensor_msgs::Imu imu;
sensor_msgs::MagneticField mag;
ros::Publisher pubimu("imu/data_raw", &imu);
ros::Publisher pubmag("imu/mag", &mag);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");

  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  // You can set your own offset for mag values
  // mpu.magXOffset = -50;
  // mpu.magYOffset = -55;
  // mpu.magZOffset = -10;

  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubimu);
  nh.advertise(pubmag);
}

void loop() {
  /**********************************************************************
   * Get IMU Data
   **********************************************************************/
  uint8_t sensorId;
  if (mpu.readId(&sensorId) == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId");
  }

  mpu.accelUpdate();
  aX = mpu.accelX();
  aY = mpu.accelY();
  aZ = mpu.accelZ();
  aSqrt = mpu.accelSqrt();
  Serial.println("accelX: " + String(aX));
  Serial.println("accelY: " + String(aY));
  Serial.println("accelZ: " + String(aZ));
  Serial.println("accelSqrt: " + String(aSqrt));

  mpu.gyroUpdate();
  gX = mpu.gyroX() + gXOffset;
  gY = mpu.gyroY() + gYOffset;
  gZ = mpu.gyroZ() + gZOffset;
  Serial.println("gyroX: " + String(gX));
  Serial.println("gyroY: " + String(gY));
  Serial.println("gyroZ: " + String(gZ));

  mpu.magUpdate();
  mX = mpu.magX();
  mY = mpu.magY();
  mZ = mpu.magZ();
  mDirection = mpu.magHorizDirection();
  Serial.println("magX: " + String(mX));
  Serial.println("maxY: " + String(mY));
  Serial.println("magZ: " + String(mZ));
  Serial.println("horizontal direction: " + String(mDirection));

  /*********************************************************************
  * ROS Publish
  **********************************************************************/
  imu.header.frame_id = "imu_link";
  imu.header.stamp = nh.now();
  imu.angular_velocity.x = gX;
  imu.angular_velocity.y = gY;
  imu.angular_velocity.z = gZ; // [rad/sec]
  imu.linear_acceleration.x = aX;      
  imu.linear_acceleration.y = aY;  
  imu.linear_acceleration.z = aZ; 
  pubimu.publish(&imu);
  
  mag.header.frame_id = "imu_link";
  mag.header.stamp = nh.now();
  mag.magnetic_field.x = mX;
  mag.magnetic_field.y = mY;
  mag.magnetic_field.z = mZ; // [μT]
  pubmag.publish(&mag);

  nh.spinOnce();
 
  delay(10);
}