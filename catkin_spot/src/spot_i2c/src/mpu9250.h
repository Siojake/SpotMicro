

#include <fcntl.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <spot_i2c/imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>




extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

// i2c addresses
#define GYR_ADDR 0x69
#define MAG_ADDR 0x0c

// register address
#define REG_PWR_MGMT_1   0x6b
#define REG_GYRO_CONFIG  0x1b
#define REG_ACEL_CONFIG1 0x1c
#define REG_ACEL_CONFIG2 0x1d
#define REG_INT_PIN_CFG  0x37
#define REG_ACEL_DATA    0x3b
#define REG_TEMP_DATA    0x41
#define REG_GYRO_DATA    0x43

#define REG_MAG_CNTL     0x0a
#define REG_MAG_ST1      0x02
#define REG_MAG_ST2      0x09
#define REG_MAG_DATX     0x03
#define REG_MAG_DATY     0x05
#define REG_MAG_DATZ     0x37

#define MODE_ONESHOT     0x11
#define MODE_100HZ       0x16 
#define MAG_MODE_PWRDOWN    0
#define MAG_MODE_SERIAL_1   1
#define MAG_MODE_SERIAL_2   2
#define MAG_MODE_SINGLE     3
#define MAG_MODE_EX_TRIGER  4
#define MAG_MODE_SELF_TEST  5

enum Grav{ G16,G8,G4 };
enum bits{ b16=16, b14=14 };
enum modes{ M8Hz,M100Hz,PWRDOWN,EXTRIGER,SINGLE };
#define ST1_DRDY        0x01

class mpu9250{
  int i2c=0;
  float gyroRange = 250.0;
  float acelRange= 2.0;
  float magRange = 4912;
  float tempSensitivity=333.87;
    
  bool  MAG_ACCESS;
  int   MAG_MODE;
  int   MAG_BIT;
  float gyroCoefficient ;
  float acelCoefficient ;
  float magCoefficient16;
  float magCoefficient14;
  float offsetRoomTemp=0;
  geometry_msgs::Vector3 offsetAccel;
  geometry_msgs::Vector3 offsetGyro;

  
public:
  int init(int fd);
  //void startAK8963();
  void configMpu();
  void resetRegister();
  void powerWakeup();
  void setMagRegister(int mode,int bit);
  void setAccelRange(int val,int _cal);
  void setGyroRange(int val,int _cal);
  void calibAccel(int count);
  void calibGyro(int count);
  int u2s(unsigned short data);
  int command (unsigned int addr, unsigned int reg, unsigned int code);
  int readByte(unsigned int addr, unsigned int reg);
  int readWord(unsigned int addr, unsigned int reg);
  int magRead(geometry_msgs::Vector3 *magData);
  int gyrRead(geometry_msgs::Vector3 *gyrData);
  int aclRead(geometry_msgs::Vector3 *aclData);
  void getGyro(geometry_msgs::Vector3 * gyro);
  void getAccel(geometry_msgs::Vector3 * acel);
  void getMag(geometry_msgs::Vector3 * mag);
  void getTemp(float * temp);
};

