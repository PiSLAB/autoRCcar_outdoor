typedef struct {
 float AccX;
 float AccY;
 float AccZ;
 float GyroX;
 float GyroY;
 float GyroZ;
 float temp;
 unsigned int sTtag;     
}IMU;

typedef struct {
 float PosX;
 float PosY;
 float PosZ;
 float VelX;
 float VelY;
 float VelZ;
 float pAcc;
 float sAcc;
 unsigned int iTow;     
}GNSS_ECEF;

