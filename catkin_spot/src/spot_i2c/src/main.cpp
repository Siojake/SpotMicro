#include "lcd.h"
#include "adc.h"
#include "mpu9250.h"
#include "tof.h"



LCD * aqm=NULL;
Adc * adc=NULL;
mpu9250 * mpu= NULL;
Tof * tof=NULL;


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
    mpu=new mpu9250();
    tof=new Tof();
    
    aqm->init(fd);
    adc->init(fd);
    mpu->init(fd);
    tof->init(fd);

   
    aqm->command(aqm->_clear);
    aqm->command(aqm->_home);
    aqm->writeLCD("Initializing...");
    ros::init(argc, argv ,"spot_I2c" );
    ros::NodeHandle nh ;
    ros::Subscriber lcdsub = nh.subscribe("/spot/lcd",  1000 , lcdCallback);
    ros::Subscriber lcd1   = nh.subscribe("/spot/lcd1", 1000 , lcd1Callback);
    ros::Subscriber lcd2   = nh.subscribe("/spot/lcd2", 1000 , lcd2Callback);
    ros::Publisher  adcpub   = nh.advertise<std_msgs::Float32> ( "/spot/battery", 1000);
    ros::Publisher  mpupub  = nh.advertise<spot_i2c::imu>     ( "/spot/imu",1000);
    ros::Publisher  tofpub   = nh.advertise<spot_i2c::tof>     ( "/spot/tof",1000);
    ros::Rate loop_rate(15);
    int  count=0;

    mpu->resetRegister();
    mpu->powerWakeup();
    aqm->command(aqm->_2ndLine);
    aqm->writeLCD("Accel");
    mpu->setAccelRange(8,true);
    aqm->command(aqm->_2ndLine);
    aqm->writeLCD("Gyro ");
    
    mpu->setGyroRange(1000,true);
    mpu->setMagRegister(M8Hz,b16);
    aqm->command(aqm->_clear);
    aqm->writeLCD("SpotMicro");

    
    while(ros::ok()){

      if ((count % 15) == 0){
	std_msgs::Float32 msg1; 
	float batvol=adc->volts();
	msg1.data=(float)batvol;
	adcpub.publish(msg1);

	float dist;
	spot_i2c::tof distance;
	
	dist=tof->distance(tof->SideLeft);
	distance.left.data=dist;

	dist=tof->distance(tof->SideRight);
	distance.right.data=dist;
	
	tofpub.publish(distance);
      }

      if ((count % 5 )==0){
	// mpu read
	spot_i2c::imu imu_raw;
	//	printf("\na");
	mpu->getAccel(&imu_raw.acel);
	//	printf(" g");
	mpu->getGyro(&imu_raw.gyro);
	//	printf(" m");
	mpu->getMag(&imu_raw.magn);
	//	printf("\n");
	mpupub.publish(imu_raw);

      }
      
      
      count++;
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }else
    return 1;
}
