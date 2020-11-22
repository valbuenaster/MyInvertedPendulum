
#include<Wire.h>
#include<math.h>

#define MPU6050_SELF_TEST_X         0x0D
#define MPU6050_SELF_TEST_Y         0x0E
#define MPU6050_SELF_TEST_Z         0x0F

#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_MOT_THR          0x1F

#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
/*
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F

#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
*/
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
/*
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
*/
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C

#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74

const int MPU_addr=0x68;
unsigned char readingRegister;
unsigned char XA_TEST;
unsigned char YA_TEST;
unsigned char ZA_TEST;
unsigned char XG_TEST;
unsigned char YG_TEST;
unsigned char ZG_TEST;

double FT_XG;
double FT_YG;
double FT_ZG;
double FT_XG_perc;
double FT_YG_perc;
double FT_ZG_perc;


int16_t AcXST,AcYST,AcZST,TmpST,GyXST,GyYST,GyZST;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void readRegister(int Register)
{
    readingRegister = 0;
    Wire.beginTransmission(MPU_addr);
    Wire.write(Register);
    Wire.endTransmission();
    Wire.requestFrom(MPU_addr,1,true);
    
    readingRegister = (0x000000FF & (Wire.read()));
}

void writeRegister(int Register,unsigned char Info)
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(Register);
    Wire.write(Info);
    Wire.endTransmission();
}

void gatherDataSelfTEST_Gyros()
{
  Serial.println("\nSelf-test routing gyroscopes...\nNormal conf.\n");
  
  readRegister( MPU6050_SELF_TEST_X);
  Serial.print("MPU6050_SELF_TEST_X");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  XA_TEST = (readingRegister  & 0b11100000)>>5;
  XG_TEST = readingRegister  & 0b00011111;
  Serial.print("XA_TEST = 0b");
  Serial.println(XA_TEST,BIN);
  Serial.print("XG_TEST = 0b");
  Serial.println(XG_TEST,BIN);
  
  readRegister( MPU6050_SELF_TEST_Y);
  Serial.print("MPU6050_SELF_TEST_Y");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  YA_TEST = (readingRegister  & 0b11100000)>>5;
  YG_TEST = readingRegister  & 0b00011111;
  Serial.print("YA_TEST = 0b");
  Serial.println(YA_TEST,BIN);
  Serial.print("YG_TEST = 0b");
  Serial.println(YG_TEST,BIN);
  
  readRegister( MPU6050_SELF_TEST_Z);
  Serial.print("MPU6050_SELF_TEST_Z");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  ZA_TEST = (readingRegister  & 0b11100000)>>5;
  ZG_TEST = readingRegister  & 0b00011111;
  Serial.print("ZA_TEST = 0b");
  Serial.println(ZA_TEST,BIN);
  Serial.print("ZG_TEST = 0b");
  Serial.println(ZG_TEST,BIN);

  if(XG_TEST == 0)
  {
    FT_XG = 0;
  }else{
    FT_XG = 3275*pow(1.046,XG_TEST-1);
  }
  
  if(YG_TEST == 0)
  {
    FT_YG = 0;
  }else{
    FT_YG = -3275*pow(1.046,YG_TEST-1);
  } 
  
  if(ZG_TEST == 0)
  {
    FT_ZG = 0;
  }else{
    FT_ZG = 3275*pow(1.046,ZG_TEST-1);
  }
   
  writeRegister(MPU6050_RA_GYRO_CONFIG,0b00000000);

  Serial.println("\nNow reading...\n");
  
  readRegister(MPU6050_RA_GYRO_CONFIG);
  Serial.print("MPU6050_RA_GYRO_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_GYRO_XOUT_H);
  Serial.print("MPU6050_RA_GYRO_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyX = (readingRegister<<8);

  readRegister( MPU6050_RA_GYRO_XOUT_L);
  Serial.print("MPU6050_RA_GYRO_XOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyX |= readingRegister; 

  readRegister( MPU6050_RA_GYRO_YOUT_H);
  Serial.print("MPU6050_RA_GYRO_YOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyY = (readingRegister<<8);

  readRegister( MPU6050_RA_GYRO_YOUT_L);
  Serial.print("MPU6050_RA_GYRO_YOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyY |= readingRegister;

  readRegister( MPU6050_RA_GYRO_ZOUT_H);
  Serial.print("MPU6050_RA_GYRO_ZOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyZ = (readingRegister<<8);

  readRegister( MPU6050_RA_GYRO_ZOUT_L);
  Serial.print("MPU6050_RA_GYRO_ZOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN); 
  GyZ |= readingRegister;
  
  Serial.println("\nWriting a register...\n");
  writeRegister(MPU6050_RA_GYRO_CONFIG,0b11100000);

  Serial.println("\nNow reading...ST conf.\n");
  
  readRegister(MPU6050_RA_GYRO_CONFIG);
  Serial.print("MPU6050_RA_GYRO_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  Serial.println("  ");
  
  readRegister( MPU6050_RA_GYRO_XOUT_H);
  Serial.print("MPU6050_RA_GYRO_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyXST = (readingRegister<<8);

  readRegister( MPU6050_RA_GYRO_XOUT_L);
  Serial.print("MPU6050_RA_GYRO_XOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyXST |= readingRegister;

  readRegister( MPU6050_RA_GYRO_YOUT_H);
  Serial.print("MPU6050_RA_GYRO_YOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyYST = (readingRegister<<8);

  readRegister( MPU6050_RA_GYRO_YOUT_L);
  Serial.print("MPU6050_RA_GYRO_YOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyYST |= readingRegister;

  readRegister( MPU6050_RA_GYRO_ZOUT_H);
  Serial.print("MPU6050_RA_GYRO_ZOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  GyZST = (readingRegister<<8);

  readRegister( MPU6050_RA_GYRO_ZOUT_L);
  Serial.print("MPU6050_RA_GYRO_ZOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN); 
  GyZST |= readingRegister; 

  writeRegister(MPU6050_RA_GYRO_CONFIG,0b00000000);
  
  if(FT_XG != 0)
  {
    FT_XG_perc = (GyXST - GyX)/FT_XG;
  }else
  {
    FT_XG_perc = (GyXST - GyX);
  }
   
  if(FT_YG != 0)
  {
    FT_YG_perc = (GyYST - GyY)/FT_YG;
  }else
  {
    FT_YG_perc = (GyYST - GyY);
  }
     
  if(FT_ZG != 0)
  {
    FT_ZG_perc = (GyZST - GyZ)/FT_ZG;
  }else
  {
    FT_ZG_perc = (GyZST - GyZ);
  }
  Serial.print("\nTRIM VALUES are...\n");
  
  Serial.print("\nFT_XG = ");
  Serial.println(FT_XG);
  Serial.print("\nFT_YG = ");
  Serial.println(FT_YG);
  Serial.print("\nFT_ZG = ");
  Serial.println(FT_ZG);
  
  Serial.print("\nPercentages are...\n");
  
  Serial.print("\nFT_XG_perc = ");
  Serial.println(FT_XG_perc);
  Serial.print("\nFT_YG_perc = ");
  Serial.println(FT_YG_perc);
  Serial.print("\nFT_ZG_perc = ");
  Serial.println(FT_ZG_perc);
}

void gatherDataSelfTEST_Accels()
{
  Serial.println("\nSelf-test routing accelerometers...\nNormal conf.\n");
  
  readRegister( MPU6050_SELF_TEST_X);
  Serial.print("MPU6050_SELF_TEST_X");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  XA_TEST = (readingRegister  & 0b11100000)>>5;
  XG_TEST = readingRegister  & 0b00011111;
  Serial.print("XA_TEST = 0b");
  Serial.println(XA_TEST,BIN);
  Serial.print("XG_TEST = 0b");
  Serial.println(XG_TEST,BIN);
  
  readRegister( MPU6050_SELF_TEST_Y);
  Serial.print("MPU6050_SELF_TEST_Y");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  YA_TEST = (readingRegister  & 0b11100000)>>5;
  YG_TEST = readingRegister  & 0b00011111;
  Serial.print("YA_TEST = 0b");
  Serial.println(YA_TEST,BIN);
  Serial.print("YG_TEST = 0b");
  Serial.println(YG_TEST,BIN);
  
  readRegister( MPU6050_SELF_TEST_Z);
  Serial.print("MPU6050_SELF_TEST_Z");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  ZA_TEST = (readingRegister  & 0b11100000)>>5;
  ZG_TEST = readingRegister  & 0b00011111;
  Serial.print("ZA_TEST = 0b");
  Serial.println(ZA_TEST,BIN);
  Serial.print("ZG_TEST = 0b");
  Serial.println(ZG_TEST,BIN);

  if(XA_TEST == 0)
  {
    FT_XG = 0;
  }else{
    FT_XG = 1392.4*pow(2.7058,(XA_TEST-1)/30);
  }
  
  if(YA_TEST == 0)
  {
    FT_YG = 0;
  }else{
    FT_YG = 1392.4*pow(2.7058,(YA_TEST-1)/30);
  } 
  
  if(ZA_TEST == 0)
  {
    FT_ZG = 0;
  }else{
    FT_ZG = 1392.4*pow(2.7058,(ZA_TEST-1)/30);
  }
   
  writeRegister(MPU6050_RA_ACCEL_CONFIG,0b00010000);//+-8g

  Serial.println("\nNow reading...\n");
  
  readRegister(MPU6050_RA_ACCEL_CONFIG);
  Serial.print("MPU6050_RA_ACCEL_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister(MPU6050_RA_ACCEL_XOUT_H);
  Serial.print("MPU6050_RA_ACCEL_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcX = (readingRegister<<8);

  readRegister(MPU6050_RA_ACCEL_XOUT_H);
  Serial.print("MPU6050_RA_ACCEL_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcX |= readingRegister; 

  readRegister(MPU6050_RA_ACCEL_YOUT_H);
  Serial.print("MPU6050_RA_ACCEL_YOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcY = (readingRegister<<8);

  readRegister(MPU6050_RA_ACCEL_YOUT_L);
  Serial.print("MPU6050_RA_ACCEL_YOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcY |= readingRegister;

  readRegister(MPU6050_RA_ACCEL_ZOUT_H);
  Serial.print("MPU6050_RA_ACCEL_ZOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcZ = (readingRegister<<8);

  readRegister(MPU6050_RA_ACCEL_ZOUT_L);
  Serial.print("MPU6050_RA_ACCEL_ZOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN); 
  AcZ |= readingRegister;
  
  Serial.println("\nWriting a register...\n");
  writeRegister(MPU6050_RA_ACCEL_CONFIG,0b11110000);

  Serial.println("\nNow reading...ST conf.\n");
  
  readRegister(MPU6050_RA_ACCEL_CONFIG);
  Serial.print("MPU6050_RA_ACCEL_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  Serial.println("  ");
  
  readRegister(MPU6050_RA_ACCEL_XOUT_H);
  Serial.print("MPU6050_RA_ACCEL_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcXST = (readingRegister<<8);

  readRegister(MPU6050_RA_ACCEL_XOUT_L);
  Serial.print("MPU6050_RA_ACCEL_XOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcXST |= readingRegister;

  readRegister(MPU6050_RA_ACCEL_YOUT_H);
  Serial.print("MPU6050_RA_ACCEL_YOUT_H ");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcYST = (readingRegister<<8);

  readRegister(MPU6050_RA_ACCEL_YOUT_L);
  Serial.print("MPU6050_RA_ACCEL_YOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcYST |= readingRegister;

  readRegister(MPU6050_RA_ACCEL_ZOUT_H);
  Serial.print("MPU6050_RA_ACCEL_ZOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  AcZST = (readingRegister<<8);

  readRegister(MPU6050_RA_ACCEL_ZOUT_L);
  Serial.print("MPU6050_RA_ACCEL_ZOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN); 
  AcZST |= readingRegister; 

  writeRegister(MPU6050_RA_ACCEL_CONFIG,0b00000000);
  
  if(FT_XG != 0)
  {
    FT_XG_perc = (AcXST - AcX)/FT_XG;
  }else
  {
    FT_XG_perc = (AcXST - AcX);
  }
   
  if(FT_YG != 0)
  {
    FT_YG_perc = (AcYST - AcY)/FT_YG;
  }else
  {
    FT_YG_perc = (AcYST - AcY);
  }
     
  if(FT_ZG != 0)
  {
    FT_ZG_perc = (AcZST - AcZ)/FT_ZG;
  }else
  {
    FT_ZG_perc = (AcZST - AcZ);
  }
  Serial.print("\nTRIM VALUES are...\n");
  
  Serial.print("\nFT_XG = ");
  Serial.println(FT_XG);
  Serial.print("\nFT_YG = ");
  Serial.println(FT_YG);
  Serial.print("\nFT_ZG = ");
  Serial.println(FT_ZG);
  
  Serial.print("\nPercentages are...\n");
  
  Serial.print("\nFT_XG_perc = ");
  Serial.println(FT_XG_perc);
  Serial.print("\nFT_YG_perc = ");
  Serial.println(FT_YG_perc);
  Serial.print("\nFT_ZG_perc = ");
  Serial.println(FT_ZG_perc);
}

void printWholeStatus()
{
  Serial.print("\nRegisters\n\n");
  
  readRegister( MPU6050_SELF_TEST_X);
  Serial.print("MPU6050_SELF_TEST_X");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  XA_TEST = (readingRegister  & 0b11100000)>>5;
  XG_TEST = readingRegister  & 0b00011111;
  Serial.print("XA_TEST = 0b");
  Serial.println(XA_TEST,BIN);
  Serial.print("XG_TEST = 0b");
  Serial.println(XG_TEST,BIN);
  
  readRegister( MPU6050_SELF_TEST_Y);
  Serial.print("MPU6050_SELF_TEST_Y");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  YA_TEST = (readingRegister  & 0b11100000)>>5;
  YG_TEST = readingRegister  & 0b00011111;
  Serial.print("YA_TEST = 0b");
  Serial.println(YA_TEST,BIN);
  Serial.print("YG_TEST = 0b");
  Serial.println(YG_TEST,BIN);
  
  readRegister( MPU6050_SELF_TEST_Z);
  Serial.print("MPU6050_SELF_TEST_Z");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  ZA_TEST = (readingRegister  & 0b11100000)>>5;
  ZG_TEST = readingRegister  & 0b00011111;
  Serial.print("ZA_TEST = 0b");
  Serial.println(ZA_TEST,BIN);
  Serial.print("ZG_TEST = 0b");
  Serial.println(ZG_TEST,BIN);
  
  readRegister( MPU6050_RA_SMPLRT_DIV);
  Serial.print("MPU6050_RA_SMPLRT_DIV");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_CONFIG  );
  Serial.print("MPU6050_RA_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_GYRO_CONFIG);
  Serial.print("MPU6050_RA_GYRO_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_ACCEL_CONFIG);
  Serial.print("MPU6050_RA_ACCEL_CONFIG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_MOT_THR );
  Serial.print("MPU6050_RA_MOT_THR");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_FIFO_EN );
  Serial.print("MPU6050_RA_FIFO_EN");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_MST_CTRL);
  Serial.print("MPU6050_RA_I2C_MST_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  /*
  readRegister( MPU6050_RA_I2C_SLV0_ADDR);
  Serial.print("MPU6050_RA_I2C_SLV0_ADDR");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV0_REG);
  Serial.print("MPU6050_RA_I2C_SLV0_REG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV0_CTRL);
  Serial.print("MPU6050_RA_I2C_SLV0_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV1_ADDR);
  Serial.print("MPU6050_RA_I2C_SLV1_ADDR");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV1_REG);
  Serial.print("MPU6050_RA_I2C_SLV1_REG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV1_CTRL);
  Serial.print("MPU6050_RA_I2C_SLV1_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV2_ADDR);
  Serial.print("MPU6050_RA_I2C_SLV2_ADDR");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV2_REG);
  Serial.print("MPU6050_RA_I2C_SLV2_REG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV2_CTRL);
  Serial.print("MPU6050_RA_I2C_SLV2_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV3_ADDR);
  Serial.print("MPU6050_RA_I2C_SLV3_ADDR");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV3_REG);
  Serial.print("MPU6050_RA_I2C_SLV3_REG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV3_CTRL);
  Serial.print("MPU6050_RA_I2C_SLV3_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  
  readRegister( MPU6050_RA_I2C_SLV4_ADDR);
  Serial.print("MPU6050_RA_I2C_SLV4_ADDR");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV4_REG);
  Serial.print("MPU6050_RA_I2C_SLV4_REG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV4_DO);
  Serial.print("MPU6050_RA_I2C_SLV4_DO");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV4_CTRL);
  Serial.print("MPU6050_RA_I2C_SLV4_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV4_DI);
  Serial.print("MPU6050_RA_I2C_SLV4_DI");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  */
  readRegister( MPU6050_RA_I2C_MST_STATUS);
  Serial.print("MPU6050_RA_I2C_MST_STATUS");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_INT_PIN_CFG);
  Serial.print("MPU6050_RA_INT_PIN_CFG");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_INT_ENABLE);
  Serial.print("MPU6050_RA_INT_ENABLE");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_INT_STATUS);
  Serial.print("MPU6050_RA_INT_STATUS");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_ACCEL_XOUT_H);
  Serial.print("MPU6050_RA_ACCEL_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_ACCEL_XOUT_L);
  Serial.print("MPU6050_RA_ACCEL_XOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_ACCEL_YOUT_H);
  Serial.print("MPU6050_RA_ACCEL_YOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_ACCEL_YOUT_L);
  Serial.print("MPU6050_RA_ACCEL_YOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_ACCEL_ZOUT_H);
  Serial.print("MPU6050_RA_ACCEL_ZOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_ACCEL_ZOUT_L);
  Serial.print("MPU6050_RA_ACCEL_ZOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_TEMP_OUT_H);
  Serial.print("MPU6050_RA_TEMP_OUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_TEMP_OUT_L);
  Serial.print("MPU6050_RA_TEMP_OUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_GYRO_XOUT_H);
  Serial.print("MPU6050_RA_GYRO_XOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_GYRO_XOUT_L);
  Serial.print("MPU6050_RA_GYRO_XOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_GYRO_YOUT_H);
  Serial.print("MPU6050_RA_GYRO_YOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_GYRO_YOUT_L);
  Serial.print("MPU6050_RA_GYRO_YOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_GYRO_ZOUT_H);
  Serial.print("MPU6050_RA_GYRO_ZOUT_H");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_GYRO_ZOUT_L);
  Serial.print("MPU6050_RA_GYRO_ZOUT_L");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  /*
  readRegister( MPU6050_RA_I2C_SLV0_DO);
  Serial.print("MPU6050_RA_I2C_SLV0_DO");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV1_DO);
  Serial.print("MPU6050_RA_I2C_SLV1_DO");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV2_DO);
  Serial.print("MPU6050_RA_I2C_SLV2_DO");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_I2C_SLV3_DO);
  Serial.print("MPU6050_RA_I2C_SLV3_DO");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);
  */
  readRegister( MPU6050_RA_I2C_MST_DELAY_CTRL);
  Serial.print("MPU6050_RA_I2C_MST_DELAY_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_SIGNAL_PATH_RESET);
  Serial.print("MPU6050_RA_SIGNAL_PATH_RESET");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_MOT_DETECT_CTRL);
  Serial.print("MPU6050_RA_MOT_DETECT_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_USER_CTRL);
  Serial.print("MPU6050_RA_USER_CTRL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_PWR_MGMT_1);
  Serial.print("MPU6050_RA_PWR_MGMT_1");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_PWR_MGMT_2);
  Serial.print("MPU6050_RA_PWR_MGMT_2");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_FIFO_COUNTH);
  Serial.print("MPU6050_RA_FIFO_COUNTH");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_FIFO_COUNTL);
  Serial.print("MPU6050_RA_FIFO_COUNTL");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  readRegister( MPU6050_RA_FIFO_R_W);
  Serial.print("MPU6050_RA_FIFO_R_W");
  Serial.print("  ");
  Serial.println(readingRegister,BIN);

  
}



void setup() 
{
  Serial.begin(38400);  
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)  
  Wire.endTransmission();

  readingRegister = 0;
  AcXST = 0;
  AcYST = 0;
  AcZST = 0;
  TmpST = 0;
  GyXST = 0;
  GyYST = 0;
  GyZST = 0;
  AcX = 0;
  AcY = 0;
  AcZ = 0;
  Tmp = 0;
  GyX = 0;
  GyY = 0;
  GyZ = 0;
  FT_XG = 0;
  FT_YG = 0;
  FT_ZG = 0;

  FT_XG_perc = 0;
  FT_YG_perc = 0;
  FT_ZG_perc = 0;

  for(char aa = 1;aa <= 12; aa++)
  {
    Serial.print("Waiting for ");
    Serial.print(aa,DEC);
    Serial.println(" seconds..."); 
    _delay_ms(1000); 
  }

  printWholeStatus();

  gatherDataSelfTEST_Gyros();
  gatherDataSelfTEST_Accels();
  
}

void loop() 
{
  // put your main code here, to run repeatedly:

}
