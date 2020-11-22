#define alphaZ 0.03
#define alpha_angvel_X 0.02

int16_t AcZ;
int16_t GyX;

int32_t  MEAN_Z = 400;//536
int32_t  MEAN_VANG_X = 833;//883


int16_t F_CompY,F_CompZ,F_AngVel_X;
int16_t CompY,CompZ,AngVel_X;
int16_t F_CompY_1,F_CompZ_1,F_AngVel_X_1;

void setup() 
{
    Serial.begin(38400);
  
    AcZ = 118;
    GyX = 173;
    F_CompZ_1 = 60;
    F_AngVel_X_1 = 250;

    CompZ = int16_t(AcZ - MEAN_Z);
    AngVel_X = int16_t(GyX - MEAN_VANG_X);

    F_CompZ = int16_t((alphaZ*CompZ) +((1-alphaZ)*F_CompZ_1));
    F_AngVel_X = int16_t((alpha_angvel_X*AngVel_X) +((1-alpha_angvel_X)*F_AngVel_X_1));

    F_CompZ_1 = F_CompZ;
    F_AngVel_X_1 = F_AngVel_X;
    
    Serial.print("\nCompZ = "); 
    Serial.println(CompZ);
    
    Serial.print("\nAngVel_X = "); 
    Serial.println(AngVel_X);
    
    Serial.print("\nF_CompZ = "); 
    Serial.println(F_CompZ);
    
    Serial.print("\nF_AngVel_X = "); 
    Serial.println(F_AngVel_X);
}

void loop() {
  // put your main code here, to run repeatedly:

}
