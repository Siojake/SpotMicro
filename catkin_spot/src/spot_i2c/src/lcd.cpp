#include "lcd.h"



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
