#include <bit>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>
#include "sensor_data.hpp"

using namespace std;

extern IMU imu = {0, };
extern GNSS_ECEF ECEF = {0, };

typedef enum {
    None = 0, 
    gyroZ = 5,
    rear_left_wheel_ticks = 8,
    rear_right_wheel_ticks,
    single_tick,
    speed,
    gyro_temp,
    gyroY,
    gyroX,
    accX = 16,
    accY,     
    accZ,
} Ublox_Sensor_datatype;

float  Intbit24Tofloat(vector<char> data, unsigned int idx) 
{
      int value = (((int)data[idx+2]) << 24) & 0xff000000;
      value |= (((int)data[idx+1]) << 16) & 0x00ff0000;
      value |= (((int)data[idx+0]) << 8) & 0x0000ff00;
      value = pow(2, -8)*value;
      
      return (float)value;
}

float  Intbit32Tofloat(vector<char> data, unsigned int idx) 
{
      int value = (((int)data[idx+3]) << 24) & 0xff000000;
      value |= (((int)data[idx+2]) << 16) & 0x00ff0000;
      value |= (((int)data[idx+1]) << 8) & 0x0000ff00;
      value |= (((int)data[idx+0])     ) & 0x000000ff;
      
      return (float)value;
}

uint32_t  byteswap32(uint32_t x)
{
      uint32_t y = (x >> 24) & 0xff;
      y |= ((x >> 16) & 0xff) << 8;
      y |= ((x >> 8) & 0xff) << 16;
      y |= (x & 0xff) << 24;
      
      return y;
}

char UBLOXDataCheckSum(vector<char> Packet, size_t Length){

    char Flag = 0;
    char CK_A = 0x0, CK_B = 0x0;
    char CheckSumDataA = Packet[6 + Length];    //CheckSum DataA
    char CheckSumDataB = Packet[6 + Length+1];    //CheckSum DataB

    // Starting from the class to before Checksum
    for(size_t i = 2; i < 6 + Length; i++){
        CK_A += Packet[i];
        CK_B += CK_A;
    }
    CK_A = (CK_A & 0xFF);
    CK_B = (CK_B & 0xFF);
    if( (CheckSumDataA == CK_A) && (CheckSumDataB == CK_B) )     Flag = 1;

    return Flag;
}

char Ublox_8_DEQUEUE(unsigned int* packet, size_t Sz)
{
	//unsigned int packet_v[Sz] = {0, }; // c++ doesn't allow VLA
	vector<unsigned int> packet_v(Sz);
	vector<char> payload(Sz*4); // This payload contains all packet of data(Head class id length payload checkA and B)
	size_t Length;
	char flag = 0;

	for(size_t i = 0; i < Sz; ++i)
		{
			packet_v[i] = packet[i]; // array to vector, 다른 방법 있을지도?
		}
	memcpy(&payload[0], &packet_v[0], Sz);
	
	// header class id로 분류 시작, header class id는 나중에 class로 선언해서 따로 묶어두고 불러오는 방식으로 코드짜면 훨씬 보기 좋음. 급해서 코드 작성은 안함.	
	if( packet_v[0] == 0x031062b5 ) // UBX-ESF-RAW(IMU)	
	{	
		unsigned int num;
		unsigned int data_type;
		unsigned int idx = 4;
		Length = (size_t)(payload[idx+1]*pow(2, 8) + payload[idx]);
		idx += 2;
		
		num = (Length-4)/8;
		//reserved 4byte
		idx += 4;
		
		for(size_t i = 0; i < num; ++i)
		{
			data_type = payload[idx+3];
			
			switch (data_type)
   			{
       			case gyroX:
       				 imu.GyroX = pow(2, -12)*Intbit24Tofloat(payload, idx)*M_PI/180;
            			break;
       			case gyroY:
       				 imu.GyroY = pow(2, -12)*Intbit24Tofloat(payload, idx)*M_PI/180;
            			break;
       			case gyroZ:
       				 imu.GyroZ = pow(2, -12)*Intbit24Tofloat(payload, idx)*M_PI/180;
            			break;
       			case accX:
       				 imu.AccX = pow(2, -10)*Intbit24Tofloat(payload, idx);
            			break;
       			case accY:
       				 imu.AccY = pow(2, -10)*Intbit24Tofloat(payload, idx);
            			break;
       			case accZ:
       				 imu.AccZ = pow(2, -10)*Intbit24Tofloat(payload, idx);
       			break;
       			case gyro_temp:
       				 imu.temp = 0.01*Intbit24Tofloat(payload, idx);
            			break;	            				
			}
			
			imu.sTtag = (unsigned int)( ((int)(payload[idx+7]) << 24) + ((int)(payload[idx+6]) << 16) + ((int)(payload[idx+5]) << 8) + ((int)(payload[idx+4]) << 0) );
						
			idx += 8;
		}
		if ( UBLOXDataCheckSum(payload, Length) )
		{
		}
		
		flag = 1;	 		
	}
	else if( packet_v[0] == 0x010162b5 ) // UBX-NAV-POSEFEF
	{	
		unsigned int idx = 4;
		Length = (size_t)(payload[idx+1]*pow(2, 8) + payload[idx]);
		idx += 2; 
		
		ECEF.iTow  = (unsigned int)( ((int)(payload[idx+3]) << 24) + ((int)(payload[idx+2]) << 16) + ((int)(payload[idx+1]) << 8) + ((int)(payload[idx+0]) << 0) ) ;
		idx += 4; 		
		ECEF.PosX = 0.01*Intbit32Tofloat(payload, idx);
		idx += 4;
		ECEF.PosY = 0.01*Intbit32Tofloat(payload, idx);
		idx += 4;		
		ECEF.PosZ = 0.01*Intbit32Tofloat(payload, idx);
		idx += 4;		
		ECEF.pAcc  = 0.01*(float)(unsigned int)( ((int)(payload[idx+3]) << 24) + ((int)(payload[idx+2]) << 16) + ((int)(payload[idx+1]) << 8) + ((int)(payload[idx+0]) << 0) ) ;
		
		if ( UBLOXDataCheckSum(payload, Length) )
		{
		}
		/*
		cout << "                        "<< endl;
		std::cout<< std::dec << iTow << endl;
		std::cout<< std::dec << ecefX << endl;
		std::cout<< std::dec << pAcc << endl;
		cout << "                        "<< endl;
		*/
		flag = 2;
	}
	else if( packet_v[0] == 0x110162b5 ) // UBX-NAV-VELECEF
	{	
		unsigned int idx = 4;		
		Length = (size_t)(payload[idx+1]*pow(2, 8) + payload[idx]);
		idx += 2; 		
		
		ECEF.iTow  = (unsigned int)( ((int)(payload[idx+3]) << 24) + ((int)(payload[idx+2]) << 16) + ((int)(payload[idx+1]) << 8) + ((int)(payload[idx+0]) << 0) ) ;
		idx += 4; 		
		ECEF.VelX = 0.01*Intbit32Tofloat(payload, idx);
		idx += 4;
		ECEF.VelY = 0.01*Intbit32Tofloat(payload, idx);
		idx += 4;		
		ECEF.VelZ = 0.01*Intbit32Tofloat(payload, idx);
		idx += 4;		
		ECEF.sAcc  = 0.01*(float)(unsigned int)( ((int)(payload[idx+3]) << 24) + ((int)(payload[idx+2]) << 16) + ((int)(payload[idx+1]) << 8) + ((int)(payload[idx+0]) << 0) ) ;
		
		if ( UBLOXDataCheckSum(payload, Length) )
		{
		}
		flag = 3;		
	}
	
	return flag;
}



