//ARDUINO MEGA RUNS AT 16 MHZ 

#include<Wire.h>
#include<math.h>

#define MEAN_Y -16542
#define MEAN_Z -1008
#define MEAN_VANG_X -316
#define alphaY 0.03
#define alphaZ 0.03
#define alpha_Angle 0.03
#define alpha_angvel_X 0.02

#define AngleRef -0.02
#define angVelRef 0.0

#define Kp 80
#define Kd 0.5

#define Maximum 16383*(Kp+Kd)

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t F_CompY,F_CompZ,F_AngVel_X;
int16_t CompY,CompZ,AngVel_X;
int16_t F_CompY_1,F_CompZ_1,F_AngVel_X_1;

double ang;
double ang_1;

double Angle;
int16_t VelAngle;

const int MPU_addr=0x68;// I2C address of the MPU-6050

int stringValue[11] = {0, 102, 204, 306, 409, 511, 613, 716, 818, 920, 1023};

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
  
  TCCR1A |= 0b11110011;
  TCCR1B |= 0b00000011;
  //ICR1 = 65535;
  ICR1 = 1023;
  
  F_CompY_1 = 0;
  F_CompZ_1 = 0;
  F_AngVel_X_1 = 0;
  ang_1 = 0;   
  
  man_Signal = 0;


/*    
  Serial.print("M_PI_2 "); 
  Serial.println(M_PI_2,4); 
  Serial.println("\nangle = "); 
  for(unsigned char ii = 0;ii<10;ii++)
  {
      //ang = Sign(zp[ii])*fabs(M_PI_2  - fabs(atan2(zp[ii],yp[ii])));
      ang = M_PI_2  - fabs(atan2(zp[ii],yp[ii]));
      Serial.print(ang,4); 
      Serial.print(", "); 
  }
*/    
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
    CompY = AcY - MEAN_Y;
    CompZ = AcZ - MEAN_Z;
    AngVel_X = GyX - MEAN_VANG_X;

    F_CompY = (alphaY*CompY) +((1-alphaY)*F_CompY_1);
    F_CompZ = (alphaZ*CompZ) +((1-alphaZ)*F_CompZ_1);
    F_AngVel_X = (alpha_angvel_X*AngVel_X) +((1-alpha_angvel_X)*F_AngVel_X_1);

    if(CompZ == 0)
    {
      ang = 0.0;
    }else
    {
      ang = Sign(F_CompZ)*fabs(M_PI_2  - fabs(atan2(F_CompZ,F_CompY)));
    }
    
    Angle = ang*alpha_Angle + (1-alpha_Angle)*ang_1;
    VelAngle = F_AngVel_X;

    F_CompY_1 = F_CompY;
    F_CompZ_1 = F_CompZ;
    F_AngVel_X_1 = F_AngVel_X;
    ang_1 = Angle;           
}

void writeMotorA(int value)
{
  PORTC &= ~((1<<5)|(1<<7));
  if(value == 0)
  {
    PORTC = 0;
  }
  
  if(value > 0)
  {
    //PORTC |= (1<<7);
    //PORTC &= ~(1<<5);
    PORTC = 0b10000000;
  }else
  { 
    //PORTC &= ~(1<<7);
    //PORTC |= (1<<5);
    PORTC = 0b00100000;
  }
  OCR1A = 1023 - fabs(value);
}

void writeMotorB(int value)
{
  PORTA &= ~((1<<4)|(1<<6)); 
  if(value == 0)
  {
    PORTA = 0;
  }
  
  if(value >= 0)
  {
    //PORTA |= (1<<7);
    //PORTA &= ~(1<<5);
    PORTA = 0b01000000;
  }else
  { 
    //PORTA |= (1<<5);
    //PORTA &= ~(1<<7);
    PORTA = 0b00010000;
  }
  OCR1B = 1023 - fabs(value);
}

int calculatecorrectionSignal()
{
  //(Kp+Kd)
  //#define AngleRef 0.0
  //#define angVelRef 0.0
  //return Kp*(AngleRef - Angle);
  //return Kd*(angVelRef - VelAngle);
  return (Kp*(AngleRef - Angle)) + (Kd*(angVelRef - VelAngle));
}

void printMatlabFormat()
{
    /*
    Serial.print(AcY);
    Serial.print(", "); 
    Serial.print(AcZ);
    Serial.print(", "); 
    Serial.print(GyX);
    Serial.println(";");  
    */   
    Serial.print(Angle,4);
    Serial.print(", "); 
    Serial.print(VelAngle);
    Serial.print(", "); 
    Serial.print(man_Signal);
    Serial.println(";");  
}

void loop() 
{
  // put your main code here, to run repeatedly:
       
    readAccelerometerMine();
    prepareReadings();
    man_Signal = calculatecorrectionSignal();
    
    writeMotorA(man_Signal);
    writeMotorB(man_Signal);
    
         
    printMatlabFormat();
   
    delay(1);
    /*
    if(counter == 20)
    {
      writeMotorA(-stringValue[ci]);
      writeMotorB(stringValue[ci]);
      counter = 1;
      
      Serial.print("ci = ");
      Serial.println(ci);
            
      if(sense == false)
      {
        ci = ci + 1;
      }else
      {
        ci = ci - 1;
      }

      if((ci == 11) && (sense == false))
      {
        ci = 9;
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

/*
ISR(TIMER0_COMPA_vect)
{

}
*/
