#include "ros/ros.h"
#include "std_msgs/String.h" 
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include <stdlib.h>
#include <stdio.h>

// REST 1:320,2:480,3:180,5:310,6:167,7:455,9:310,10:474,11:180,13:308,14:175,15:450

struct servo{
  int no;
  int minAngle;
  int maxAngle;
  int minAbs;
  int maxAbs;
};
servo servos[]={
  { 1,-45, 45,416,220},  // RL1
  { 2,-20, 90,526,285},  // RL2
  { 3, 30,180,182,509},  // RL3
  
  { 5,-45, 45,217,414},  // RR1
  { 6,-20, 90,124,364},  // RR2
  { 7, 30,180,455,128},  // RR3

  { 9,-45, 45,214,410},  // FL1
  {10,-20, 90,514,274},  // FL2
  {11, 30,180,188,515},  // FL3

  {13,-45, 45,408,212},  // FR1
  {14,-20, 90,138,379},  // FR2
  {15, 30,180,443,116}   // FR3
};

int convert(int ch, int deg,int * out){
  int ret=-1;
  for (int i=0;i<12;i++){
    if (servos[i].no == ch){
      if ((servos[i].minAngle <= deg) && (deg <= servos[i].maxAngle)){ 
	float adiff=(servos[i].maxAngle-servos[i].minAngle);
	float vdiff=(servos[i].maxAbs-servos[i].minAbs);
	float d1=(vdiff/adiff)*(float)((deg-servos[i].minAngle))+servos[i].minAbs;
	*out= (int ) d1;
	ret=1;
	break;
      }
    }
  }
  return ret; 
}
  
ros::Publisher rp;
//i2cpwm_board::ServoArray * parse(char * buf ,i2cpwm_board::ServoArray * sa){
int parse(char * buf ,i2cpwm_board::ServoArray * sa){
  char *p;
  //  i2cpwm_board::ServoArray sa;
  char *s[20];
  int n=0;
  int ret=0;
  p=strtok(buf,",");
  while( p  != NULL){
    s[n]=p;
    n++;
    p=strtok(NULL,",");
  }
  ret=0;
  if (n>0){
    sa->servos.resize(n);
    int ch, deg,out;
    for (int i=0;i<n;i++){
      if (s[i][0] != 'a'){
	p=strtok(s[i],":");
	//      while( p !=0){
	ch=atoi(p);
	p=strtok(NULL,":");
	deg=atoi(p);
	if (convert(ch,deg,&out)>0){

	  sa->servos[i].servo=ch;
	  sa->servos[i].value=out;
	  ret++;

	}else{
	  return -1;
	}
      }else{ //if (*st!='a'){
	// absolut mode
	p=strtok( &s[i][1],":");
	ch=atoi(p);
	p=strtok( NULL,":");
	deg=atoi(p);
	sa->servos[i].servo=ch;
	sa->servos[i].value=deg;
	ret++;	 
      }
    }
  }else{
    ret=-1;
  }
    return ret;
}


int main(int argc,char ** argv){
  ros::init(argc,argv,"servo_angle_node");
  ros::NodeHandle nh;
  i2cpwm_board::ServoArray sa;
  
  rp=nh.advertise<i2cpwm_board::ServoArray> ("/servos_absolute",1000);
  char buf[2048];
  ros::Rate loop_rate(1);
  while( ros::ok()){
    printf("ch:val,ch:val....#");
    fgets(buf,2048,stdin);
    int ret=parse(buf, &sa);
    if (ret>0){
      rp.publish(sa);
    }
    ros::spinOnce();
  }
}
