
#ifndef ADC_H
#define ADC_H
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>


extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}


class Adc {
 
  int i2c;
  float Vref  = 2.048;
  float multi = 5.00;
  unsigned short addr=0x68;
  unsigned short config;
public:
  void command(int addr,int code);
  void init(int fd);
  float volts(void);
};


#endif

