//ARDUINO MEGA RUNS AT 16 MHZ 

#include<Wire.h>
#include<math.h>

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

int stringValue[41] = {0, 102, 204, 306, 409, 511, 613, 716, 818, 920, 1023, 920, 818, 716, 613, 511, 409, 306, 204, 102, 0,-102, -204, -306, -409, -511, -613, -716, -818, -920, -1023, -920, -818, -716, -613, -511, -409, -306, -204, -102, 0};
int stSTVL;

char counter = 1;
int ci = 0;
bool sense = false;

int man_Signal;

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
    int counter = 0;
    int32_t  sumV_Z = 0;
    int32_t  sum_VANG_X = 0;

    while (counter < 500)
    {
      readAccelerometerMine();
      sumV_Z = sumV_Z + AcZ;
      sum_VANG_X = sum_VANG_X + GyX;
      delay(1);
      counter = counter + 1;
    } 
    MEAN_Z =  int32_t(sumV_Z/500);//536
    MEAN_VANG_X = int32_t(sum_VANG_X/500);//883
}

void setup() 
{
  // put your setup code here, to run once:
  noInterrupts();
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
  interrupts();
  
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

  //Wait for some time so the accelerometer settles
  sensorZeroing();//WE NEED TO WAIT FOR 12 SECONDS! AT LEAST!
  //delay(5000);   
}

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

void prepareReadings()
{
    CompZ = int16_t(AcZ - MEAN_Z);
    AngVel_X = int16_t(GyX - MEAN_VANG_X);

    F_CompZ = (alphaZ*CompZ) +((1-alphaZ)*F_CompZ_1);
    F_AngVel_X = (alpha_angvel_X*AngVel_X) +((1-alpha_angvel_X)*F_AngVel_X_1);

    F_CompZ_1 = F_CompZ;
    F_AngVel_X_1 = F_AngVel_X;
}

void writeMotorA(int value)
{
  //PORTC &= ~((1<<5)|(1<<7));
  //if(Sign(value) != Sign(pwmAprev))
  //{
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
  //}
  //OCR1A = 1023 - fabs(value);
  OCR1A = fabs(value)+90;
  pwmAprev = value;
}

void writeMotorB(int value)
{
  //PORTA &= ~((1<<4)|(1<<6)); 
  
    Serial.print("value, "); 
    Serial.println(value,DEC);
  
  //if(Sign(value) != Sign(pwmBprev))
  //{
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
  //}
  //OCR1B = 1023 - fabs(value);
  OCR1B = fabs(value)+90 ;
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
    
    Serial3.print(AcY);
    Serial3.print(", "); 
    Serial3.print(AcZ);
    Serial3.print(", "); 
    Serial3.print(GyX);
    Serial3.println(";");  
     
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
  // put your main code here, to run repeatedly:
       
    readAccelerometerMine();
    prepareReadings();
    man_Signal = calculatecorrectionSignal();
    
    writeMotorA(man_Signal);
    writeMotorB(man_Signal);
      
       
    //writeMotorA(150);
    //writeMotorB(150);

     
    //printMatlabFormat();
    
    //printBlueTooth(GyX,MEAN_VANG_X,F_AngVel_X);
    //printBlueTooth(AcZ,MEAN_Z,F_CompZ);
    //printBlueTooth(MEAN_Z,MEAN_VANG_X,1234);
    
    printBlueTooth(F_CompZ,F_AngVel_X,man_Signal);//when normal use

    delay(1);
    /*
    int ii = 0;
    
    while(1)
    {
      stSTVL = stringValue[ii];
      //writeMotorA(stSTVL);
      writeMotorB(-stSTVL);

      if(ii == 41)
      {
        ii = 0;
      }else
      {
        ii = ii + 1;
      }
      delay(1000);
    }
    */
    /*
    if(counter == 40)
    {
      stSTVL = stringValue[ci];
      //writeMotorA(stSTVL);
      writeMotorB(-stSTVL);
      counter = 1;
      
      //Serial.print("ci = ");
      //Serial.println(ci);
            
      if(sense == false)
      {
        ci = ci + 1;
      }else
      {
        ci = ci - 1;
      }

      if((ci == 11) && (sense == false))
      {
        ci = 19;
        sense = true;
      }

      if((ci == -1) && (sense == true))
      {
        ci = 1;
        sense = false;
      }
      
    }else
    {
      counter = counter + 1;
    }   
    */
}

