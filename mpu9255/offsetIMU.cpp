#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
using namespace std;

int main(int argc, char **argv){

  int fd; 
  wiringPiSetupSys();
  fd = wiringPiI2CSetup(0x68);

  int16_t InBuffer[9] = {0}; 
  float giro_x=0,giro_y=0, giro_z=0;
  float acc_x=0,acc_y=0, acc_z=0;

  ofstream myfile;
  myfile.open (argv[1]);
  
  float conversion_giro = 1/32.8f;
  float conversion_acce = 9.8/16384.0f;

  for (int i=0; i< 1000;i++){
    //datos acelerómetro
    InBuffer[0]=  (wiringPiI2CReadReg8 (fd, 0x3B)<<8)|wiringPiI2CReadReg8 (fd, 0x3C);
    InBuffer[1]=  (wiringPiI2CReadReg8 (fd, 0x3D)<<8)|wiringPiI2CReadReg8 (fd, 0x3E);
    InBuffer[2]=  (wiringPiI2CReadReg8 (fd, 0x3F)<<8)|wiringPiI2CReadReg8 (fd, 0x40);   
    
    acc_x = acc_x + InBuffer[0];
    acc_y = acc_y + InBuffer[1];
    acc_z = acc_z + InBuffer[2];

    //datos giroscopio
    InBuffer[3]=  (wiringPiI2CReadReg8 (fd, 0x43)<<8)|wiringPiI2CReadReg8 (fd, 0x44);
    InBuffer[4]=  (wiringPiI2CReadReg8 (fd, 0x45)<<8)|wiringPiI2CReadReg8 (fd, 0x46);
    InBuffer[5]=  (wiringPiI2CReadReg8 (fd, 0x47)<<8)|wiringPiI2CReadReg8 (fd, 0x48); 

    giro_x = giro_x + InBuffer[3];
    giro_y = giro_y + InBuffer[4];
    giro_z = giro_z + InBuffer[5]; 

  }
  giro_x = (giro_x / 1000 ) * conversion_giro;
  giro_y = (giro_y / 1000 ) * conversion_giro;
  giro_z = (giro_z / 1000 ) * conversion_giro;

  acc_x = (acc_x / 1000 ) * conversion_acce;
  acc_y = (acc_y / 1000 ) * conversion_acce;
  acc_z = (acc_z / 1000 ) -16384 ;


  myfile << giro_x << "," << giro_y << "," << giro_z << "," << acc_x << "," << acc_y << "," << acc_z * conversion_acce << endl;
  myfile.close();

  return 0;
}
