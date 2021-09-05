#include "mpu9250.h"
#include <stdio.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}



void mpu9250::resetRegister(){

    command(MAG_ADDR,0x0b,0x01);
    command(GYR_ADDR,0x6b,0x80);
    MAG_ACCESS=false;
    usleep(10000);

}

void mpu9250::powerWakeup(){

  command(GYR_ADDR,REG_PWR_MGMT_1  ,0x00);
  command(GYR_ADDR,REG_INT_PIN_CFG ,0x02);
  MAG_ACCESS=true;

}


void mpu9250::setMagRegister(int mode,int bit){
  int _wrtData;
  if (MAG_ACCESS == false){
    return; 
  }
  int _wrrData =0;
  switch(mode){
  case M8Hz:
    _wrtData=0x02; 
    MAG_MODE= MAG_MODE_SERIAL_1;
    break;
  case M100Hz:
    _wrtData=0x06;
    MAG_MODE= MAG_MODE_SERIAL_2;
    break;
  case PWRDOWN:
    _wrtData=0x00;
    MAG_MODE= MAG_MODE_PWRDOWN;
    break;
  case EXTRIGER:
    _wrtData=0x04;
    MAG_MODE= MAG_MODE_EX_TRIGER;
    break;
  case SINGLE:
    _wrtData=0x01;
    MAG_MODE=MAG_MODE_SINGLE;
  }
  switch(bit){
  case b14:
    _wrtData=_wrtData | 0x00;
    MAG_BIT=14;
    break;
   
  case b16:
    _wrtData= _wrtData | 0x10;
    MAG_BIT=16;
    
  }
  command(MAG_ADDR,0x0a,_wrtData);
  
}

    
void mpu9250::setAccelRange(int val ,int _cal){
  int _data;
  switch(val){
  case G16:
    acelRange=16;
    _data=0x18;
    break;
    
  case G8:
    acelRange=8;
    _data=0x10;
    break;
  case G4:
    acelRange=4;
    _data=0x08;
    break;
    
  default:
    acelRange=2;
    _data=0x00;
    break;
  }

  command(GYR_ADDR,REG_ACEL_CONFIG1,_data);
  acelCoefficient =  acelRange /(float)0x8000;
  offsetAccel.x=0;
  offsetAccel.y=0;
  offsetAccel.z=0;
  
  if (_cal){
    calibAccel(1000);
  }
}


void mpu9250::setGyroRange(int val ,int _cal){
  int _data;
  int gyroRange;
  
  switch(val){
  case 2000:
    gyroRange=2000;
    _data= 0x18;
    break;
  case 1000:
    gyroRange=1000;
    _data= 0x10;
    break;
  case 500:
    gyroRange=500;
    _data=0x08;
    break;
  default:
    gyroRange=250;
    _data=0x00;

  }
  command(GYR_ADDR,REG_GYRO_CONFIG,_data);
  gyroCoefficient= gyroRange/ (float)0x8000;
  usleep(10000);
  offsetGyro.x=0;
  offsetGyro.y=0;
  offsetGyro.z=0;
  if (_cal){
    calibGyro(1000);
  }
}

int mpu9250::u2s(unsigned short data){
  if (data &(0x01 <<15)){
    return -1 * ((data ^ 0xffff)+1);
  }
  return (int)data;
}

void mpu9250::getAccel(geometry_msgs::Vector3 * acel){
  unsigned short x,y,z;
  x=readWord(GYR_ADDR,0x3b);
  y=readWord(GYR_ADDR,0x3d);
  z=readWord(GYR_ADDR,0x3f);

  acel->x=acelCoefficient * u2s(x);
  acel->y=acelCoefficient * u2s(y);
  acel->z=acelCoefficient * u2s(z);
}

void mpu9250::getGyro(geometry_msgs::Vector3 * gyro){
  unsigned short x,y,z;
  x=readWord(GYR_ADDR,0x43);
  y=readWord(GYR_ADDR,0x45);
  z=readWord(GYR_ADDR,0x47);

  gyro->x=gyroCoefficient * u2s(x);
  gyro->y=gyroCoefficient * u2s(y);
  gyro->z=gyroCoefficient * u2s(z);

}

void mpu9250::getMag(geometry_msgs::Vector3 * mag){
  int status;
  if (MAG_ACCESS != true){
    mag->x=0;
    mag->y=0;
    mag->z=0;
    return;
  }
  int _wrtData;
  switch (MAG_MODE){
    case MAG_MODE_SINGLE:
      if (MAG_BIT == 14){
	_wrtData=0x01;
      }else{
	_wrtData=0x11;
      }
      command(MAG_ADDR,0x0a,_wrtData);
      usleep(10000);
      break;
  case MAG_MODE_SERIAL_1:
  case MAG_MODE_SERIAL_2:
    status=readByte(MAG_ADDR,0x02);
    if ((status&0x02) == 0x02){
      readByte(MAG_ADDR,0x09);
    }
    break;
  case MAG_MODE_EX_TRIGER:
  case MAG_MODE_PWRDOWN:
    break;
  }

  unsigned char st=readByte(MAG_ADDR,0x02);
  while(( st& 0x01) !=0x01) {
    usleep(10*1000);
    st=readByte(MAG_ADDR,0x02);
  }
  
  unsigned short x,y,z;
  unsigned char h,l;
  // x=readWord(MAG_ADDR,0x03);
  l=readByte(MAG_ADDR,0x03);
  h=readByte(MAG_ADDR,0x04);
  x=h*0x100+l;
  //y=readWord(MAG_ADDR,0x05);
  l=readByte(MAG_ADDR,0x05);
  h=readByte(MAG_ADDR,0x06);
  y=h*0x100+l;
  //z=readWord(MAG_ADDR,0x07);
  l=readByte(MAG_ADDR,0x07);
  h=readByte(MAG_ADDR,0x08);
  z=h*0x100+l;
  int st2=readByte(MAG_ADDR,0x09);
  

  if ((st2&0x08)==0x08){
    printf("overflow");    
  }
  if (MAG_BIT==16){
  
    mag->x=u2s(x)*magCoefficient16;
    mag->y=u2s(y)*magCoefficient16;
    mag->z=u2s(z)*magCoefficient16;
  }else{
    mag->x=u2s(x)*magCoefficient14;
    mag->y=u2s(y)*magCoefficient14;
    mag->z=u2s(z)*magCoefficient14;
  }
  
}

void mpu9250::getTemp(float *tmp){
  *tmp=0.0;
  int dat=readWord(GYR_ADDR,0x65);
  *tmp=((dat - offsetRoomTemp)/tempSensitivity)+21;
}



void mpu9250::calibAccel(int count=1000){
  geometry_msgs::Vector3 sum;
  geometry_msgs::Vector3 data;
  sum.x=0;
  sum.y=0;
  sum.z=0;
  
  for (int i=0;i<count ; i++){
    getAccel(&data);
    sum.x +=data.x;
    sum.y +=data.y;
    sum.z +=data.z;
  }

  offsetAccel.x= -1.0 * sum.x/(float)count;
  offsetAccel.y= -1.0 * sum.y/(float)count;
  offsetAccel.z= -1.0 * sum.z/(float)count;
}

void mpu9250::calibGyro(int count=1000){
  geometry_msgs::Vector3 sum;
  geometry_msgs::Vector3 data;
  sum.x=0;
  sum.y=0;
  sum.z=0;
  
  for (int i=0;i<count ; i++){
    getGyro(&data);
    sum.x +=data.x;
    sum.y +=data.y;
    sum.z +=data.z;
  }

  offsetGyro.x= -1.0 * sum.x/(float)count;
  offsetGyro.y= -1.0 * sum.y/(float)count;
  offsetGyro.z= -1.0 * sum.z/(float)count;
}

int mpu9250::init(int fd){
  i2c=fd;
  command(GYR_ADDR,REG_PWR_MGMT_1,0x80);
  command(GYR_ADDR,REG_PWR_MGMT_1,0x00);
  command(GYR_ADDR,REG_INT_PIN_CFG,0x02);
  command(GYR_ADDR,REG_ACEL_CONFIG1,0x08);


  gyroCoefficient = gyroRange/(float)0x8000;
  acelCoefficient = acelRange/(float)0x8000;
  magCoefficient16= magRange/(float)32760.0;
  magCoefficient14= magRange/(float)8190.0;
  

  geometry_msgs::Vector3 aclCal;
  aclCal.x=0;
  aclCal.y=0;
  aclCal.z=0;
    
  for ( int i=0 ;i<1000 ; i++){
    geometry_msgs::Vector3 dat;
    aclRead(&dat);
    aclCal.x=aclCal.x+dat.x;
    aclCal.y=aclCal.y+dat.y;
    aclCal.z=aclCal.z=dat.z;
  }
  offsetAccel.x=-1.0* aclCal.x/1000.0;
  offsetAccel.y=-1.0* aclCal.y/1000.0;
  offsetAccel.z=-1.0* ((aclCal.z/1000.0) -1.0);
  
  return i2c;

};


int mpu9250::command(unsigned int addr,unsigned int reg,unsigned int code){
  int ret;
  if (i2c !=0){
    ioctl(i2c,I2C_SLAVE,addr);
    ret=i2c_smbus_write_byte_data(i2c,reg,code);
  }
  usleep(1000);
  return ret;
}

int mpu9250::readByte(unsigned int addr,unsigned int reg){
  int ret;
  if (i2c !=0){
    ioctl(i2c,I2C_SLAVE,addr);
    ret=i2c_smbus_read_byte_data(i2c,reg);
  }
  return ret;
}


int mpu9250::readWord(unsigned int addr,unsigned int reg){
  int ret=-1;
  union {
    unsigned short s;
    unsigned char b[2];
  }dat;
  
  
  if (i2c !=0){
    ioctl(i2c,I2C_SLAVE,addr);
    //    printf("addr:%x ",addr);
    
    dat.s=i2c_smbus_read_word_data(i2c,reg);
    //printf("dat:%02x %02x ",dat.b[0],dat.b[1]);
    int h= dat.b[0];
    int l= dat.b[1];
    //int h= i2c_smbus_read_byte_data(i2c,reg);
    //int l= i2c_smbus_read_byte_data(i2c,reg+1);
    
    ret= h*0x100+l;
    //    printf("%02x %02x",h,l);
    
    if (ret & (1<<15)){
         ret= -1 * ((ret ^ 0xffff) +1);
    }
   }
  return ret;
}


int mpu9250::magRead(geometry_msgs::Vector3 * magData){
  // read mak8963 status
  unsigned char st1,st2;
  int i;
  #define LOOP_MAX 20
  st1=0;
  st2=0;
  for(i=0;i<LOOP_MAX;i++){
    st1=readByte(MAG_ADDR,REG_MAG_ST1);
    st2=readByte(MAG_ADDR,REG_MAG_ST2);
    if (st1 & ST1_DRDY){
      return -1; 
    }
  }
  magData->x=readWord(MAG_ADDR,REG_MAG_DATX);
  magData->y=readWord(MAG_ADDR,REG_MAG_DATY);
  magData->z=readWord(MAG_ADDR,REG_MAG_DATZ);

  return 1; 
  
}


int mpu9250::gyrRead(geometry_msgs::Vector3 * gyrData){
  gyrData->x=readWord(GYR_ADDR,REG_GYRO_DATA);
  gyrData->y=readWord(GYR_ADDR,REG_GYRO_DATA+2);
  gyrData->z=readWord(GYR_ADDR,REG_GYRO_DATA+4);
  return 1;
}

int mpu9250::aclRead(geometry_msgs::Vector3 * aclData){
  
  aclData->x=readWord(GYR_ADDR,REG_ACEL_DATA);
  aclData->y=readWord(GYR_ADDR,REG_ACEL_DATA+2);
  aclData->z=readWord(GYR_ADDR,REG_ACEL_DATA+4);

  
  aclData->x = (8.0/(float)0x8000) * aclData->x;
  aclData->y = (8.0/(float)0x8000) * aclData->y;
  aclData->z = (8.0/(float)0x8000) * aclData->x;
  
  return 1;
}
