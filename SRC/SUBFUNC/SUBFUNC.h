#pragma once



////****  using NEWMAT  ****////
////
#define NOMINMAX

#define WANT_MATH
//#define WANT_STREAM
#include "../NEWMAT10D/newmatap.h"
//#include "NEWMAT10D/newmatio.h"
#include "../NEWMAT10D/precisio.h"

#ifdef use_namespace
using namespace NEWMAT;
#endif






//**********************************************************************//
//****************                                   *******************//
//****************   performance(time) check macro   *******************//
//****************                                   *******************//
//**********************************************************************//


//#define TIME_UNIT (1.0/3600) // hour
//#define TIMEUNIT "[hour]"

//#define TIME_UNIT (1.0/60) // minute
//#define TIMEUNIT "[minutes]"

//#define TIME_UNIT (1.0) // second
//#define TIMEUNIT "[seconds]"

//#define TIME_UNIT	(1000.0)	// milli-sec
//#define TIMEUNIT	"[milli-sec]"

#define TIME_UNIT (1000000.0)	// micro-sec
#define TIMEUNIT "[micro-sec]"

//#define TIME_UNIT (1000000000.0)	// nano-sec
//#define TIMEUNIT "[nano-sec]"


//----------  processing time check : eps = micro-seconds
#define TIME_START(s, f) \
{ \
	QueryPerformanceFrequency( (LARGE_INTEGER*)&f ); \
	QueryPerformanceCounter( (LARGE_INTEGER*)&s ); \
} \

#define TIME_COPY(s, d) \
{ \
	d = s; \
} \

#define TIME_END(p, c, e, f) \
{ \
	QueryPerformanceCounter( (LARGE_INTEGER*)&c ); \
	e = ((double)(c - p) / (double)f) * TIME_UNIT; \
	p = c; \
} \

#define TIME_ELAPSED(_ts, _tc, _te, _fr) \
{ \
	QueryPerformanceCounter( (LARGE_INTEGER*)&_tc ); \
	_te = ((double)(_tc - _ts) / (double)_fr) * TIME_UNIT; \
	_ts = _tc; \
} \
// _tu : 1.0 for second, 1000.0 for mili-second, 1000000.0 for micro-second

//----------  processing time check : eps = milli-seconds
#define time_start(s) \
{ \
	s = clock(); \
} \

#define time_copy(s, d) \
{ \
	d = s; \
} \


#define time_end(p, c, e) \
{ \
	c = clock(); \
	e = (double)(c - p)*TIME_UNIT / CLOCKS_PER_SEC; \
	p = c; \
} \
	//







//*************************************************************************//
//****************                                      *******************//
//****************    UTC vs GPST vs LOCAL Conversion   *******************//
//****************                                      *******************//
//*************************************************************************//
void GetUTC(struct tm *gmt);
void GetLocalTime(struct tm* st);
int GetGPSTime(int year,int month,int day, int hour,int minute,double second,double *gpstime);
int GetLeapSecond();
int GetWeekNumber();

// 1:utc->local, 2:utc->gpst, 3:local->utc, 4:local->gpst, 5:gpst->utc, 6:gpst->local
//void UTCvsLocalTvsGPST(int nMode);

typedef struct{

	int leapsec;
	int year;
	int month;
	int day;
	int hour;
	int minute;
	double sec;

}type_utc;

// GAFAS에서 가져옴.
void UTC2GPStime(type_utc &utc,double &gpst, int &week, int &leapsecond);
void GPStime2UTC(double gpst, int week, type_utc *utc);
void GPStime2UTC(double gpst, int week, int leapsecond, type_utc *utc);
int GetGPSTfrmUTC(int year,int month,int day, int hour,int minute,double second,double *gpstime, int &leapsec);
int  GetLeapSecondUTC(int year, int month, int mday,
	int hour = 0, int minute = 0, double second = 0);
//int  GetGPSWeekNo(int year, int month, int mday, int hour, int minute, double second);
int  GetGPSWeekNo(const double long_gpst);



//***************************************************************//
//****************                            *******************//
//****************    LLH format Conversion   *******************//
//****************                            *******************//
//***************************************************************//

double NMEA0183ToDegree(double nmea);
double DegreeToNMEA0183(double deg);
double DMSToDegree(double dms);
double DegreeToDMS(double deg);








//***********************************************************************//
//********************                            ***********************//
//********************    Coordinate Conversion   ***********************//
//********************                            ***********************//
//***********************************************************************//

//===Choice 1: WGS-84 parameters===//
#define epam	6378137.0000			// earth semimajor axis in meters
#define epbm	6356752.3142			// earth semiminor axis in meters
#define epe		8.181919092890692e-002	// earth eccentricity, i.e. sqrt(1-(epbm/epam)*(epbm/epam));

//===Choice 2: Bessel-1841 parameters===//
//#define epam 6377397.155 // earth semimajor axis in meters. Bessel-1841
//#define epbm 6356078.962832441 // earth semiminor axis in meters. Bessel-1841
//#define epe 8.169683119526466e-002

#define pi		3.14159265358979
#define PI		pi

#define f2m		0.30479951			// conversion from feet to m
#define m2f		3.28084517			// conversion from m to feet
#define r2d		57.29577951308238	// radian to degree
#define d2r		0.01745329251994	// degree to radian

#define epaf	epam*m2f	// earth semimajor axis in ft
#define epbf	epbm*m2f	// earth semimajor axis in ft


Matrix Cxyz2enu(Matrix&  orgxyz);


Matrix elazcal(double Xs,double Ys, double Zs, double Xu, double Yu, double Zu);


Matrix enu2xyz(const Matrix& enu, const Matrix& orgxyz);
Matrix xyz2enu(const Matrix& xyz,const Matrix&  orgxyz);


Matrix llh2xyz(const Matrix& llh);
Matrix xyz2llh(const Matrix& xyz);


Matrix ned2xyz(const Matrix& ned, const Matrix& orgxyz);
Matrix xyz2ned(const Matrix& xyz, const Matrix&  orgxyz);


Matrix nedv2xyzv(const Matrix& ned, const Matrix& orgxyz);


Matrix geocenllh2xyz(const Matrix& llh);
Matrix xyz2geocenllh(const Matrix& xyz);







//***********************************************************************//
//********************                            ***********************//
//********************    etc                     ***********************//
//********************                            ***********************//
//***********************************************************************//


double norm(const Matrix& M);
double round(const double val);	// RTCM3에 정의되있지만...흠 어떻게 될랑가..충돌되나..


void PrintT(const Matrix &m);
void fPrintT(FILE* fp, const Matrix &m);
