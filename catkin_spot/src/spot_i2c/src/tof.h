
#ifndef TOF_H
#define TOF_H
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <spot_i2c/tof.h>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}


class Tof {

  int i2c;
  int gpio;
  unsigned short addr=0x52;
  unsigned short cmd=0xd3;
  void setAddress(void);  
  
public:
  Tof(void);
  enum { SideLeft,SideRight};
  void echo(char * fn,char *bu);
  void enable(int side);
  
  void init(int fd);
  int distance(int);
  
};


#endif

