//ARDUINO MEGA RUNS AT 16 MHZ 

#include<Wire.h>
#include<math.h>

#define MPU6050_SELF_TEST_X         0x0D
#define MPU6050_SELF_TEST_Y         0x0E
#define MPU6050_SELF_TEST_Z         0x0F

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

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

#define OFFSET_MOTORS 90

#define alphaY 0.03
#define alphaZ 0.03
#define alpha_Angle 0.03
#define alpha_angvel_X 0.02

#define CompZRef 0
#define angVelRef 0

#define Kp 0.041//0.042
#define Kd 0.1//0.1

int32_t  MEAN_Z = 400;//536
int32_t  MEAN_VANG_X = 833;//883

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t F_CompY,F_CompZ,F_AngVel_X;
int16_t CompY,CompZ,AngVel_X;
int16_t F_CompY_1,F_CompZ_1,F_AngVel_X_1;

int16_t pwmAprev;
int16_t pwmBprev;

double ang;
double ang_1;

double Angle;
int16_t VelAngle;

const int MPU_addr=0x68;// I2C address of the MPU-6050

unsigned char readingRegister;
int stringValue[41] = {0, 102, 204, 306, 409, 511, 613, 716, 818, 920, 1023, 920, 818, 716, 613, 511, 409, 306, 204, 102, 0,-102, -204, -306, -409, -511, -613, -716, -818, -920, -1023, -920, -818, -716, -613, -511, -409, -306, -204, -102, 0};
int stSTVL;

char counter = 1;
int ci = 0;
bool sense = false;

int man_Signal;

void readRegister(unsigned char Register)
{
    readingRegister = 0;
    Wire.beginTransmission(MPU_addr);
    Wire.write(Register);
    Wire.endTransmission();
    Wire.requestFrom(MPU_addr,1,true);
    
    readingRegister = (0x000000FF & (Wire.read()));
}

void writeRegister(unsigned char Register,unsigned char Info)
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(Register);
    Wire.write(Info);
    Wire.endTransmission();
}

int16_t readVariable(unsigned char RegH,unsigned char RegL)
{
    int16_t variable = 0;
    readRegister(RegH);
    variable = (readingRegister<<8);
    readRegister(RegL);
    variable |= readingRegister;
    return variable;
}

double Sign(double Value)
{
  double Signo;
  if(Value == 0.0)
  {
    Signo = 0.0;
  }
  if(Value > 0.0)
  {
    Signo = 1.0;
  }else
  {
    Signo = -1.0;
  }
  return Signo;
}

int Sign(int Value)
{
  int Signo;
  if(Value == 0)
  {
    Signo = 0;
  }
  if(Value > 0)
  {
    Signo = 1;
  }else
  {
    Signo = -1;
  }
  return Signo;
}

//WE NEED TO WAIT FOR 12 SECONDS! AT LEAST!
void sensorZeroing()
{
    int16_t counter = 0;
    int32_t  sumV_Z = 0;
    int32_t  sum_VANG_X = 0;

    //Settling for 12 s
    for(char aa = 1;aa <= 12; aa++)
    {
      _delay_ms(1000);
      printBlueTooth(int16_t(aa-1),int16_t(aa),int16_t(aa+1)); 
    }
    
    //Then 10 s
    while (counter < 10000)
    {
      readAccelerometerMine();
      sumV_Z = sumV_Z + AcZ;
      sum_VANG_X = sum_VANG_X + GyX;
      _delay_ms(1);
      counter = counter + 1;
    } 
    MEAN_Z =  int32_t(sumV_Z/10000);//536
    MEAN_VANG_X = int32_t(sum_VANG_X/10000);//883
}

void setup() 
{
  // put your setup code here, to run once:
  
/* 
  TCCR0A = 0b00000010;//CTC MODE
  TCCR0B = 0b00000100;//256 div factor
  OCR0A = 155;//Value to achieve 200 Hz
  //TCCR0B = 0b00000101;//256 div factor
  //OCR0A = 77;//Value to achieve 100 Hz
  TIMSK0 = 0b00000010;
*/
  Serial.begin(38400);
  Serial3.begin(38400);

  Serial.print("LLEGA AQUI");
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)  
  Wire.endTransmission();

  DDRB |= 0b01100000;
  DDRC |= 0b10100000;
  DDRA |= 0b01010000;

  PORTL = 0;
  PORTC = 0;
  
  TCCR1A |= 0b10100011;
  TCCR1B |= 0b00000001;
  //ICR1 = 65535;
  ICR1 = 1023;
  
  F_CompY_1 = 0;
  F_CompZ_1 = 0;
  F_AngVel_X_1 = 0;
  ang_1 = 0;   
  
  man_Signal = 0;
  pwmAprev = 0;
  pwmBprev = 0;
  stSTVL = 0;
  
  readingRegister = 0;

  //Wait for some time so the accelerometer settles
  sensorZeroing();//WE NEED TO WAIT FOR 12 SECONDS! AT LEAST! 
}

/*
void readAccelerometerMine()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
*/

void readAccelerometerMine()
{
    AcZ = readVariable(MPU6050_RA_ACCEL_ZOUT_H,MPU6050_RA_ACCEL_ZOUT_L);
    GyX = readVariable(MPU6050_RA_GYRO_XOUT_H,MPU6050_RA_GYRO_XOUT_L);
}

void prepareReadings()
{
    CompZ = int16_t(AcZ - MEAN_Z);
    AngVel_X = int16_t(GyX - MEAN_VANG_X);

    F_CompZ = int16_t((alphaZ*CompZ) +((1-alphaZ)*F_CompZ_1));
    F_AngVel_X = int16_t((alpha_angvel_X*AngVel_X) +((1-alpha_angvel_X)*F_AngVel_X_1));

    F_CompZ_1 = F_CompZ;
    F_AngVel_X_1 = F_AngVel_X;
}

void writeMotorA(int value)
{
    if(value == 0)
    {
      PORTC = 0;
    }
    if(value > 0)
    {
      PORTC = 0b10000000;
    }else
    { 
      PORTC = 0b00100000;
    }
    OCR1A = fabs(value) + OFFSET_MOTORS;
    pwmAprev = value;
}

void writeMotorB(int value)
{
    if(value == 0)
    {
      PORTA = 0;
    }
    
    if(value >= 0)
    {
      PORTA = 0b01000000;
    }else
    { 
      PORTA = 0b00010000;
    }
    OCR1B = fabs(value)+ OFFSET_MOTORS ;
    pwmBprev = value;
}

int calculatecorrectionSignal()
{
  //(Kp+Kd)
  //#define AngleRef 0.0
  //#define angVelRef 0.0
  //return Kp*(AngleRef - Angle);
  //return Kd*(angVelRef - VelAngle);
  return (Kp*(CompZRef - F_CompZ)) + (Kd*(angVelRef - F_AngVel_X));
}

void printMatlabFormat()
{ 
    Serial.print(AcZ);
    Serial.print(", "); 
    Serial.print(GyX);
    Serial.println(";");  
     
    /*  
    Serial.print(Angle,4);
    Serial.print(", "); 
    Serial.print(VelAngle);
    Serial.print(", "); 
    Serial.print(man_Signal);
    Serial.println(";");  
    */
}

void printBlueTooth(int16_t  val1,int16_t val2,int16_t val3)
{
    byte Ay1,Ay2,Az1,Az2,vx1,vx2;
    Ay1 = (0xFF)&(val1);
    Ay2 = (0xFF)&(val1>>8);
    Az1 = (0xFF)&(val2);
    Az2 = (0xFF)&(val2>>8);
    vx1 = (0xFF)&(val3);
    vx2 = (0xFF)&(val3>>8);
    
    Serial3.write(0x40);
    Serial3.write(Ay1);
    Serial3.write(Ay2);
    Serial3.write(Az1);
    Serial3.write(Az2);
    Serial3.write(vx1);
    Serial3.write(vx2);
    Serial3.write(0x23);
    
    /*  AngVel_X
    Serial.print(Angle,4);
    Serial.print(", "); 
    Serial.print(VelAngle);
    Serial.print(", "); 
    Serial.print(man_Signal);
    Serial.println(";");  
    */
}
void loop() 
{
    readAccelerometerMine();
    prepareReadings();
    man_Signal = calculatecorrectionSignal();
    
    writeMotorA(man_Signal);
    writeMotorB(man_Signal);

    //printMatlabFormat();
    
    //printBlueTooth(GyX,MEAN_VANG_X,F_AngVel_X);
    //printBlueTooth(AcZ,MEAN_Z,F_CompZ);
    //printBlueTooth(MEAN_Z,MEAN_VANG_X,1234);
    
    printBlueTooth(F_CompZ,F_AngVel_X,man_Signal);//when normal use

    delay(1);
}

