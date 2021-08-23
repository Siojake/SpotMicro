#include "adc.h"


void Adc::command(int addr,int code){
  if (ioctl(i2c,I2C_SLAVE,addr)<0){
    printf("can not set slave address to LCD:%x\n",addr);
    return ; 
  }
  
  i2c_smbus_write_byte_data(i2c,config,code);
  
}

void Adc::init(int fd){
  i2c=fd;
  addr=0x68;
  config =0b10011000;
  
  command(addr,config);
}


float Adc::volts(void){
  if (ioctl(i2c,I2C_SLAVE,addr)<0){
    printf("can not set slave address to LCD:%x\n",addr);
    return  -1; 
  }
  
  unsigned short data=i2c_smbus_read_word_data(i2c,config);
  unsigned char h=data>>8;
  unsigned char l=data&0xff;
  unsigned short val=l*0x100+h;
  
  float volts=Vref * val/32767.0 *multi;

  return volts;
}




// int main(int argc, char * argv[]){
//   int fd=open("/dev/i2c-1",O_RDWR);
//   if (fd>0) {
//     Adc * adc=new Adc();
//     adc->init(fd);
//     float vol=adc->volts();

//     printf("%fV\n",vol);
//   }
// }
