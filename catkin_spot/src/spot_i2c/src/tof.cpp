#include "tof.h"


Tof::Tof(){
  wiringPiSetup();
  i2c=0;
  gpio=0;
}

void Tof::init(int fd){
  
  gpio=0;
  i2c=fd;
}

void Tof::echo(char *fname, char * buf){

  int fd;
  fd=open(fname,O_WRONLY);
  if (fd<0){
    char eb[255];
    sprintf(eb, "error open(%s):",fname);
    perror(eb);
    exit(1);
  }

  int r=write(fd,buf,strlen(buf));
  if (fd<0){
    char eb[255];
    sprintf(eb, "error write (%s , %s) :",fname,buf);
    perror(eb);
    exit(1);
  }
  close(fd);
}

  

  

void Tof::enable(int side){

  // char sw[3],s;		       
  // switch(side){
  // case SideLeft:   s=0; break;
  // case SideRight:  s=1; break;
  // }
  // sprintf(sw,"%d",s);
  // char fexp[64];
  // char fdir[64];
  // char fval[64];
  // char fune[64];
  // sprintf(fexp,"/sys/class/gpio/export");
  // sprintf(fdir,"/sys/class/gpio/gpio%d/direction",18);
  // sprintf(fval,"/sys/class/gpio/gpio%d/value",18);
  // sprintf(fune,"/sys/class/gpio/unexport");

  // echo( fexp, "18" );
  // echo( fdir, "out");
  // echo( fval, sw   );
  // echo( fune, "18" );
  pinMode(18,OUTPUT);
  digitalWrite(18,side);
};

void Tof::setAddress(){
  if (i2c>0){
    ioctl(i2c,I2C_SLAVE,addr);
  }
}

int  Tof::distance(int side){

  setAddress();

  //  int d=i2c_smbus_read_word_data(i2c,0xD3);

  unsigned char b[3];
  unsigned char dum=0xd3;
  if (write(i2c,&dum,1) != 1 ){
    perror("write");
    close(i2c);
    exit(01);
  }

  if (read(i2c,&b,2) != 2) {
    perror("read");
    close(i2c);
    exit(1);
  }
      
  
  
  //  unsigned char h,l;
  //h=d >> 8;
  //l=d & 0xff;
  //int ret=l*0x100+h;

  int ret=b[0]*0x100+b[1];
  
  return ret;
}
  



// int main(int argc,char * argv[]){
//   Tof *tof=new Tof();

//   int fd=open("/dev/i2c-1",O_RDWR);
//   if (fd >0){
//     tof->init(fd);
//     for (int i=0;i<1000;i++){
//       int r,l;
//       tof->enable(tof->SideRight);
//       usleep(10*1000);
//       l=tof->distance(tof->SideLeft);

//       tof->enable(tof->SideRight);
//       usleep(10*1000);
//       r=tof->distance(tof->SideRight);
//       printf("l:%6d r:%6d\n",l,r);
//       usleep(100*1000);
//     }
//   }
// }
