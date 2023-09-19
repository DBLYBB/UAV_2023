#ifndef __ANO_DRV_UBLOX_GPS_H__
#define __ANO_DRV_UBLOX_GPS_H__

#include "SysConfig.h"

typedef struct
{
  u32 ITOW;      // ms   GPS Millisecond Time of week   U4
  s32 Longitude; // deg  Longitude                      i4 1e-7
  s32 Latitude;  // deg  Latitude                       i4 1e-7
  s32 Height;    // mm   Height above ellipsoid                      地球高度
  s32 HMSL;      // mm   Height above mean sea level                 海拔高度
  u32 HAcc;      // mm   Horizontal accuracy estimate
  u32 VAcc;      // mm   Vertical accuracy estimate
  u16 Statuds;
} UBXPOSLLH_t;

typedef struct
{
  u32 ITOW;      // ms   GPS Millisecond Time of week
  s32 FTOW;      // ns   remainder of rounded ms above
  s16 week;      // week GPS week num
  u8 GPSfix;     //      GPSfix Type ,range 0...6
  u8 Flags;      //      fix status flags
  s32 ECEF_X;    // cm   ECEF X coordinate
  s32 ECEF_Y;    // cm   ECEF Y coordinate
  s32 ECEF_Z;    // cm   ECEF Z coordinate
  u32 PAcc;      // cm   3D Position Accuracy Estimate
  s32 ECEFVX;    // cm/s ECEF X velocity
  s32 ECEFVY;    // cm/s ECEF Y velocity
  s32 ECEFVZ;    // cm/s ECEF Z velocity
  u32 SAcc;      // cm/s speed accuracy estimate
  u16 PDOP;      //      position DOP                     0.01
  u8 reserved1;  //      Reserved
  u8 numSV;      //      Number of SVs used in Nav Solution
  u32 reserved2; //      Reserved
  u16 Statuds;   //    len
} UBXSOL_t;

typedef struct
{
  u32 ITOW;        // ms   GPS time of week of the navigation epoch.
  s32 VEL_N;       // cm/s North velocity component
  s32 VEL_E;       // cm/s East velocity component
  s32 VEL_D;       // cm/s Down velocity component
  u32 Speed;       // cm/s Speed(3-D)
  u32 GroundSpeed; // cm/s Ground speed (2-D)
  s32 Heading;     // deg  Heading of motion 2-D
  u32 SAcc;        // cm/s Speed accuracy Estimate
  u32 CAcc;        // deg  Course/Heading accuracy estimate
  u16 Statuds;

} UBXVELNED_t;

typedef struct
{

  u32 ITOW;      // ms
  uint16_t gDOP; //Geometric  DOP  0.01
  uint16_t pDOP; //Position   DOP  0.01
  uint16_t tDOP; //Time       DOP  0.01
  uint16_t vDOP; //Vertical   DOP  0.01
  uint16_t hDOP; //Horizontal DOP  0.01
  uint16_t nDOP; //Northing   DOP  0.01
  uint16_t eDOP; //Easting    DOP  0.01
  u16 Statuds;   //
} UBXDOP_t;

typedef struct
{

  u32 ITOW;    // ms
  u32 TAcc;    // ns
  s32 NANO;    // ns
  u16 Year;    // y
  u8 Month;    // month
  u8 Day;      // d
  u8 Hour;     // h
  u8 Min;      // min
  u8 Sec;      // s
  u8 Valid;    //
  u16 Statuds; //
} UBXTIMEUTC_t;

typedef struct
{
  u32 iTOW; //ms
  u16 year; //y
  u8 month; //m
  u8 day;   //d
  u8 hour;
  u8 min;
  u8 sec;
  u8 valid;   //Validity Flags (see graphic below)
  u32 tAcc;   // ns Time accuracy estimate (UTC)
  s32 nano;   // ns Fraction of second, range -1e9 .. 1e9 (UTC)
  u8 fixType; //
  /*
	GNSSfix Type, range 0..5
	0x00 = No Fix
	0x01 = Dead Reckoning only
	0x02 = 2D-Fix
	0x03 = 3D-Fix
	0x04 = GNSS + dead reckoning combined
	0x05 = Time only fix
	0x06..0xff: reserved
	*/
  u8 flags;     // - Fix Status Flags (see graphic below)
  u8 reserved1; // - Reserved
  u8 numSV;     // - Number of satellites used in Nav Solution
  s32 lon;      //1e-7 deg Longitude
  s32 lat;      //1e-7 deg Latitude
  s32 height;   // mm Height above ellipsoid
  s32 hMSL;     // mm Height above mean sea level
  u32 hAcc;     // mm Horizontal accuracy estimate
  u32 vAcc;     // mm Vertical accuracy estimate
  s32 velN;     // mm/s NED north velocity
  s32 velE;     // mm/s NED east velocity
  s32 velD;     // mm/s NED down velocity
  s32 gSpeed;   // mm/s Ground Speed (2-D)
  s32 headMot;  // 1e-5 deg Heading of motion (2-D)
  u32 sAcc;     // mm/s Speed accuracy estimate
  u32 headAcc;  // 1e-5 deg Heading accuracy estimate (both motion and vehicle)
  u16 pDOP;     // 0.01 - Position DOP
  //	u16 reserved2;// - Reserved
  //	u32 reserved3;// - Reserved
  //	s32 headVeh;// deg Heading of vehicle (2-D)1e-5
  //	u32 reserved4;// - Reserved

} __attribute__((packed)) _UBXPVT_st;

#define UBX_BUF_NUM 100
union _ubx {
  u8 pvt_buf[UBX_BUF_NUM];
  _UBXPVT_st pvt_data;
};

//typedef struct
//{
//	u8 updata_cnt;
//	u8 state;//0:offline 1:online but unavailable 2: online and availabal 3:online and works in a good state
//
//	u8 fixType;//
//	u8 numSV;// - Number of satellites used in Nav Solution
//	s32 lon;//1e-7 deg Longitude
//	s32 lat;//1e-7 deg Latitude
////	s32 height;// mm Height above ellipsoid
//	s32 hMSL;// mm Height above mean sea level
//	u32 hAcc;// mm Horizontal accuracy estimate
//	u32 vAcc;// mm Vertical accuracy estimate
//	s32 velN;// mm/s NWU north velocity
//	s32 velW;// mm/s NWU west velocity
//	s32 velU;// mm/s NWU up velocity
////	s32 gSpeed;// mm/s Ground Speed (2-D)
////	s32 headMot;// 1e-5 deg Heading of motion (2-D)
//	u32 sAcc;// mm/s Speed accuracy estimate
////	u32 headAcc;// 1e-5 deg Heading accuracy estimate (both motion and vehicle)
//	u16 pDOP;// 0.01 - Position DOP
//

//}_ubx_user_data_st;
//extern _ubx_user_data_st ubx_user_data;

//
void GPS_Rate_H(void);
void GPS_Rate_L(void);
//public
void Init_GPS(void);
void GPS_Data_Prepare_Task(u8 dT_ms);
void UBLOX_M8_GPS_Data_Receive(u8 Data);

#endif
