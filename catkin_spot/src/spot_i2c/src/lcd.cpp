#include <stdio.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include "lcd.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>


#include <std_msgs/String.h>

#include "adc.h"
#include <std_msgs/Float32.h>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}


LCD * aqm=NULL;
Adc * adc=NULL;

void LCD::init(int fd){
  i2c=fd;
  command(0x38);
  command(0x39);
  command(0x14);
  command(0x73);
  command(0x56);
  command(0x6c);
  command(0x38);
  command(_clear);
  command(display_on);  
}


void LCD::command(int code){
  if (ioctl(i2c,I2C_SLAVE,addr02)<0){
    printf("can not set slave address to LCD:%x\n",addr02);
    return ; 
  }
  
  i2c_smbus_write_byte_data(i2c,_command,code);
  usleep(1000);
}

void LCD::writeLCD(char * msg){
  unsigned char length = strlen(msg);
  
  //  i2c_smbus_write_block_data(i2c,_data,length,(unsigned char *) msg);
  for (int i=0;i<length;i++){
    i2c_smbus_write_byte_data(i2c,_data,msg[i]);
  }

}


void lcdCallback(const std_msgs::String::ConstPtr & msg){
  char buf[256];

  aqm->command(aqm->_clear);
  aqm->command(aqm->_home);
  
  int len=strlen(msg->data.c_str());
  memset(buf, 0, 256);
  memmove(buf,msg->data.c_str(),len);
  char * p;
  p=strtok(buf,"\n");
  p[16]=0;
  aqm->command(aqm->_1stLine);
  aqm->writeLCD(p);
  p=strtok(NULL,"\n");
  if (p!=NULL){
    p[16]=0;
    aqm->command(aqm->_2ndLine);
    aqm->writeLCD(p);
  }
}

void lcd1Callback(const std_msgs::String::ConstPtr & msg){
  char buf[256];
  int len=strlen(msg->data.c_str());
  memset(buf, 0, 256);
  memmove(buf,msg->data.c_str(),len);
  char * p;
  p=strtok(buf,"\n");
  p[16]=0;
  aqm->command(aqm->_1stLine);
  aqm->writeLCD(p);
}

void lcd2Callback(const std_msgs::String::ConstPtr & msg){
  char buf[256];
  int len=strlen(msg->data.c_str());
  memset(buf, 0, 256);
  memmove(buf,msg->data.c_str(),len);
  char * p;
  p=strtok(buf,"\n");
  p[16]=0;
  aqm->command(aqm->_2ndLine);
  aqm->writeLCD(p);
}


int main(int argc , char * argv[]){

  int fd = open("/dev/i2c-1",O_RDWR);
  if (fd >0){
    aqm=new LCD();
    adc=new Adc();
    aqm->init(fd);
    adc->init(fd);
    aqm->command(aqm->_clear);
    aqm->command(aqm->_home);
    
    ros::init(argc, argv ,"spot_I2c" );
    ros::NodeHandle nh ;
    ros::Subscriber lcdsub = nh.subscribe("/spot/lcd",  1000 , lcdCallback);
    ros::Subscriber lcd1   = nh.subscribe("/spot/lcd1", 1000 , lcd1Callback);
    ros::Subscriber lcd2   = nh.subscribe("/spot/lcd2", 1000 , lcd2Callback);
    ros::Publisher  adcvol = nh.advertise<std_msgs::Float32> ( "/spot/battery", 1000);

    ros::Rate loop_rate(30);
    int  count=0;
    
    while(ros::ok()){

      if ((count % 15) == 0){
	std_msgs::Float32 msg1; 
	float batvol=adc->volts();
	msg1.data=(float)batvol;
	adcvol.publish(msg1);
      }

     
      
      count++;
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }else
    return 1;
}
