#ifndef LCD_H
#define LCD_H
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <std_msgs/String.h>


extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}



extern int _controller_io_handle;

class LCD{
public:
  int i2c;
  int addr02=0x3e;
  int _command=0x00;
  int _data=0x40;
  int _clear=0x01;
  int _home=0x02;
  int display_on=0x0c;
  int _2ndLine=0x40+0x80;
  int _1stLine=0x80;
  
public:
  void command(int code);
  void writeLCD(char * message);
  void init(int fd);
};


#endif
