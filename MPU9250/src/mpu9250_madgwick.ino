#include <Wire.h>
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <MPU9250_asukiaaa.h>

#define sampleFreqDef   10.0f          // sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

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

float phi   = 0.0f;
float theta = 0.0f;
float psi   = 0.0f;

float aSqrt      = 0.0f;
float mDirection = 0.0f;
float gXOffset   = 0.3f;
float gYOffset   = 1.3f;
float gZOffset   = -0.9f;

// Magwick filter
float invSampleFreq;
float beta;        // algorithm gain
float q0;
float q1;
float q2;
float q3; // quaternion of sensor frame relative to auxiliary frame
char anglesComputed;

/***********************************************************************
 * Prototype Definition
 **********************************************************************/
void Madgwick_Init();
void MadgwickAHRS_IMU(double gx, double gy, double gz, double ax, double ay, double az);
void MadgwickAHRS(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
float getRoll();
float getPitch();
float getYaw();
float getRollRadians();
float getPitchRadians();
float getYawRadians();
float invSqrt(float x);
void computeAngles();

/**********************************************************************
 * ROS Parameter
 **********************************************************************/
ros::NodeHandle nh;

sensor_msgs::Imu imu;
sensor_msgs::MagneticField mag;
ros::Publisher pubimu("imu/data_raw", &imu);
ros::Publisher pubmag("imu/mag", &mag);

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

/**********************************************************************
 * Setup
 **********************************************************************/
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");

  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  // You can set your own offset for mag values
  mpu.magXOffset = -50;
  mpu.magYOffset = -55;
  mpu.magZOffset = -10;

  //MadgwickFilter 初期化
  Madgwick_Init();
  
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubimu);
  nh.advertise(pubmag);
  tfbroadcaster.init(nh);
}

/**********************************************************************
 * loop
 **********************************************************************/
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

  // Magwick filter
  //MadgwickAHRS_IMU(gX, gY, gZ, aX, aY, aZ);
  MadgwickAHRS(gX, gY, gZ, aX, aY, aZ, mX, mY, mZ);
  
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

  imu.orientation.w = q0;
  imu.orientation.x = q1;
  imu.orientation.y = q2;
  imu.orientation.z = q3;
  pubimu.publish(&imu);
  
  mag.header.frame_id = "imu_link";
  mag.header.stamp = nh.now();
  mag.magnetic_field.x = mX;
  mag.magnetic_field.y = mY;
  mag.magnetic_field.z = mZ; // [μT]
  pubmag.publish(&mag);

  tfs_msg.header.stamp = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = q0;
  tfs_msg.transform.rotation.x = q1;
  tfs_msg.transform.rotation.y = q2;
  tfs_msg.transform.rotation.z = q3;
  tfbroadcaster.sendTransform(tfs_msg);
  
  nh.spinOnce();
 
  delay(10);
}

/**********************************************************************
 * Magwick filter
 **********************************************************************/
void Madgwick_Init() {
  beta = betaDef;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  invSampleFreq = 1.0f / sampleFreqDef;
  anglesComputed = 0;
}

void MadgwickAHRS_IMU(double gx, double gy, double gz, double ax, double ay, double az) {
   double recipNorm;
   double s0, s1, s2, s3;
   double qDot1, qDot2, qDot3, qDot4;
   double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

   // Convert gyroscope degrees/sec to radians/sec
   gx *= 0.0174533;
   gy *= 0.0174533;
   gz *= 0.0174533;

   // Rate of change of quaternion from gyroscope
   qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
   qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
   qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
   qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
   if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

     // Normalise accelerometer measurement
     recipNorm = invSqrt(ax * ax + ay * ay + az * az);
     ax *= recipNorm;
     ay *= recipNorm;
     az *= recipNorm;

     // Auxiliary variables to avoid repeated arithmetic
     _2q0 = 2.0 * q0;
     _2q1 = 2.0 * q1;
     _2q2 = 2.0 * q2;
     _2q3 = 2.0 * q3;
     _4q0 = 4.0 * q0;
     _4q1 = 4.0 * q1;
     _4q2 = 4.0 * q2;
     _8q1 = 8.0 * q1;
     _8q2 = 8.0 * q2;
     q0q0 = q0 * q0;
     q1q1 = q1 * q1;
     q2q2 = q2 * q2;
     q3q3 = q3 * q3;

     // Gradient decent algorithm corrective step
     s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
     s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
     s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
     s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
     recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
     s0 *= recipNorm;
     s1 *= recipNorm;
     s2 *= recipNorm;
     s3 *= recipNorm;

     // Apply feedback step
     qDot1 -= beta * s0;
     qDot2 -= beta * s1;
     qDot3 -= beta * s2;
     qDot4 -= beta * s3;
   }

   // Integrate rate of change of quaternion to yield quaternion
   q0 += qDot1 * invSampleFreq;
   q1 += qDot2 * invSampleFreq;
   q2 += qDot3 * invSampleFreq;
   q3 += qDot4 * invSampleFreq;

   // Normalise quaternion
   recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   q0 *= recipNorm;
   q1 *= recipNorm;
   q2 *= recipNorm;
   q3 *= recipNorm;
   anglesComputed = 0.0;
 }
 
void MadgwickAHRS(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) {
 double recipNorm;
 double s0, s1, s2, s3;
 double qDot1, qDot2, qDot3, qDot4;
 double hx, hy;
 double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

 // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
 if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
   MadgwickAHRS_IMU(gx, gy, gz, ax, ay, az);
 }

 // Convert gyroscope degrees/sec to radians/sec
 gx *= 0.0174533;
 gy *= 0.0174533;
 gz *= 0.0174533;

 // Rate of change of quaternion from gyroscope
 qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
 qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
 qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
 qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

 // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
 if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
   // Normalise accelerometer measurement
   recipNorm = invSqrt(ax * ax + ay * ay + az * az);
   ax *= recipNorm;
   ay *= recipNorm;
   az *= recipNorm;

   // Normalise magnetometer measurement
   recipNorm = invSqrt(mx * mx + my * my + mz * mz);
   mx *= recipNorm;
   my *= recipNorm;
   mz *= recipNorm;

   // Auxiliary variables to avoid repeated arithmetic
   _2q0mx = 2.0 * q0 * mx;
   _2q0my = 2.0 * q0 * my;
   _2q0mz = 2.0 * q0 * mz;
   _2q1mx = 2.0 * q1 * mx;
   _2q0 = 2.0 * q0;
   _2q1 = 2.0 * q1;
   _2q2 = 2.0 * q2;
   _2q3 = 2.0 * q3;
   _2q0q2 = 2.0 * q0 * q2;
   _2q2q3 = 2.0 * q2 * q3;
   q0q0 = q0 * q0;
   q0q1 = q0 * q1;
   q0q2 = q0 * q2;
   q0q3 = q0 * q3;
   q1q1 = q1 * q1;
   q1q2 = q1 * q2;
   q1q3 = q1 * q3;
   q2q2 = q2 * q2;
   q2q3 = q2 * q3;
   q3q3 = q3 * q3;

   // Reference direction of Earth's magnetic field
   hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
   hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
   _2bx = sqrtf(hx * hx + hy * hy);
   _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
   _4bx = 2.0 * _2bx;
   _4bz = 2.0 * _2bz;

   // Gradient decent algorithm corrective step
   s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
   s0 *= recipNorm;
   s1 *= recipNorm;
   s2 *= recipNorm;
   s3 *= recipNorm;

   // Apply feedback step
   qDot1 -= beta * s0;
   qDot2 -= beta * s1;
   qDot3 -= beta * s2;
   qDot4 -= beta * s3;

    phi   = getPhi();
    theta = getTheta();
    psi   = getPsi();
 }

 // Integrate rate of change of quaternion to yield quaternion
 q0 += qDot1 * invSampleFreq;
 q1 += qDot2 * invSampleFreq;
 q2 += qDot3 * invSampleFreq;
 q3 += qDot4 * invSampleFreq;

 // Normalise quaternion
 recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
 q0 *= recipNorm;
 q1 *= recipNorm;
 q2 *= recipNorm;
 q3 *= recipNorm;
 anglesComputed = 0.0;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

double getPhi() {
  if (!anglesComputed) computeAngles();
  return phi * 57.29578;
}
double getTheta() {
  if (!anglesComputed) computeAngles();
  return theta * 57.29578;
}
double getPsi() {
  if (!anglesComputed) computeAngles();
  return psi * 57.29578;
}
double getPhiRadians() {
  if (!anglesComputed) computeAngles();
  return phi;
}
double getThetaRadians() {
  if (!anglesComputed) computeAngles();
  return theta;
}
double getPsiRadians() {
  if (!anglesComputed) computeAngles();
  return psi;
}

void computeAngles(){
  phi = atan2f(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
  theta = -1 * asinf(-2.0 * (q1*q3 - q0*q2));
  psi = atan2f(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
  anglesComputed = 1.0;
}