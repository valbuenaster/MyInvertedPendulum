#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

byte chainChar[24] = {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'};
char bytesReceived;
char Limmit;

int16_t AcY,AcZ,GyX;

void readBuffer()
{
    char ii = 0;
    while(mySerial.available()>0)
    {
      chainChar[ii] = mySerial.read();
      ii++;
      if(ii >= 24)
      {
        break;
      }
    }
}


void setup() 
{
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  Serial.println("One serial working");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(38400);
  //mySerial.println("Hello, world?");
  bytesReceived = 0;
}

void loop() 
{ // run over and over
  bytesReceived = mySerial.available();
  if (bytesReceived > 0) 
  {
    //Serial.write(mySerial.read());
    readBuffer();
    /*
    AcY = chainChar[0] + (chainChar[1]<<8);
    AcZ = chainChar[2] + (chainChar[3]<<8);
    GyX = chainChar[4] + (chainChar[5]<<8);
    */
    
    if((chainChar[0] == 0x40)&(chainChar[7] == 0x23))
    {  
      AcY = chainChar[1] + (chainChar[2]<<8);
      AcZ = chainChar[3] + (chainChar[4]<<8);
      GyX = chainChar[5] + (chainChar[6]<<8);
  
      Serial.print(AcY);
      Serial.print(", "); 
      Serial.print(AcZ);
      Serial.print(", "); 
      Serial.print(GyX);
      Serial.println(";"); 
    }
  }
  
  if (mySerial.overflow()) 
  {
    Serial.println("SoftwareSerial overflow!");
  }
}
