//#pragma once

//#pragma warning(disable: 4100)	// unreferenced variable
#pragma warning(disable: 4101)	//참조되지 않은 지역 변수입니다.
//
//#pragma warning(disable: 4127)	// constant statement, if()
//#pragma warning(disable: 4146)	//단항 빼기 연산자가 부호 없는 형식에 적용되었습니다. 결과는 역시 unsigned입니다.
//#pragma warning(disable: 4189)	// local variable was initialized but unreferenced
//
//#pragma warning(disable: 4238)
//#pragma warning(disable: 4244)	// casting - loss of data
//#pragma warning(disable: 4245)
//#pragma warning(disable: 4267)	// casting - loss of data
//
//#pragma warning(disable: 4311)	// casting - loss of data
//#pragma warning(disable: 4334)	//'<<' : 32비트 시프트의 결과가 암시적으로 64비트로 변환됩니다. 64비트 시프트를 사용하시겠습니까?
//
//#pragma warning(disable: 4505)	// eliminate the unreferenced local function.
//#pragma warning(disable: 4512)
//
//#pragma warning(disable: 4701)	// using un-initialized local variable
//#pragma warning(disable: 4706)	// 조건식 내에 할당이 있습니다.
//
//#pragma warning(disable: 4996)	// deprecation

#pragma warning(disable: 4474)	// warning C4474: 'fprintf' : too many arguments passed for format string



//#define NOMINMAX
#include "../../stdafx.h"
#include "SUBFUNC.h"

//#include <stdio.h>	// NISL_SOLUTION_C
//#include <time.h>		// NISL_SOLUTION_C




//*************************************************************************//
//****************                                      *******************//
//****************    UTC vs GPST vs LOCAL Conversion   *******************//
//****************                                      *******************//
//*************************************************************************//

//struct tm {
//	int tm_sec;     /* seconds after the minute - [0,59] */
//	int tm_min;     /* minutes after the hour - [0,59] */
//	int tm_hour;    /* hours since midnight - [0,23] */
//	int tm_mday;    /* day of the month - [1,31] */
//	int tm_mon;     /* months since January - [0,11] */ <<<<<<<<<<<<<<<<
//	int tm_year;    /* years since 1900 */ <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//	int tm_wday;    /* days since Sunday - [0,6] */
//	int tm_yday;    /* days since January 1 - [0,365] */
//	int tm_isdst;   /* daylight savings time flag */
//};

void GetUTC(struct tm *gmt)
{	
	time_t tt = time(NULL);	
	gmtime_s(gmt, &tt);	
}

void GetLocalTime(struct tm* lt)
{
	time_t		tt = time(NULL);
	localtime_s(lt, &tt);
}

int GetGPSTime(int year,int month,int day, int hour,int minute,double second,double *gpstime)
{
	int   dayofw,dayofy, yr, ttlday, m, weekno;
	static  int  dinmth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	/* Check limits of day, month and year */
	if (year < 1981 || month < 1 || month > 12 || day < 1 || day > 31)
		weekno = 0;

	/*  Convert day, month and year to day of year */
	if (month == 1)
		dayofy = day;
	else
	{
		dayofy = 0;
		for (m=1; m<=(month-1); m++)
		{
			dayofy += dinmth[m];
			if ( m==2 )
			{
				if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0)
					dayofy += 1;
			}
		}
		dayofy += day;
	}

	/*  Convert day of year and year into week number and day of week */
	ttlday = 360;
	for (yr=1981; yr<=(year-1); yr++)
	{
		ttlday  += 365;
		if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 ==0)
			ttlday  += 1;
	}
	ttlday += dayofy;
	weekno  = ttlday/7;
	dayofw  = ttlday - 7 * weekno;

	//*gpstime =  (hour * 3600 + minute * 60 + second);// + dayofw * 86400);
	//*gpstime =  (second + minute*60 + hour*3600 + dayofw*86400 + weekno*SECONDS_IN_WEEK);
	*gpstime =  (second + minute*60 + hour*3600 + dayofw*86400 + weekno*604800.0);

	return weekno;
}

int	GetLeapSecond()
{
	struct tm utc;
	GetUTC(&utc);

	int syear = 1950;	//MJD, Modified Julian Date
	int cyear = utc.tm_year + 1900;	

	int n4 = 0, n100 = 0, n400 = 0;
	for (int k = syear; k <= cyear; ++k){
		if ( k%4 == 0 )		++n4;	// true
		if ( k%100 == 0 )	++n100;	// false
		if ( k%400 == 0 )	++n400;	// true						
	}
	return n4 - n100 + n400;	
}

int GetWeekNumber()
{
	struct tm utc;
	GetUTC(&utc);
	double gpstime;
	return GetGPSTime(utc.tm_year+1900, utc.tm_mon+1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec, &gpstime);
}
// 1:utc->local, 2:utc->gpst, 3:local->utc, 4:local->gpst, 5:gpst->utc, 6:gpst->local
//void UTCvsLocalTvsGPST(int nMode)
//{
//
//}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
void UTC2GPStime(type_utc &utc,double &gpst, int &week, int &leapsecond)
{
	int year = utc.year;
	int month = utc.month;
	int day = utc.day;
	int hour = utc.hour;
	int minute = utc.minute;
	double second = utc.sec;

	int   dayofw,dayofy, yr, ttlday, m, weekno;
	static  int  dinmth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	///* Check limits of day, month and year */
	if (year < 1981 || month < 1 || month > 12 || day < 1 || day > 31)
		weekno = 0;

	///*  Convert day, month and year to day of year */
	if (month == 1)
		dayofy = day;
	else
	{
		dayofy = 0;
		for (m=1; m<=(month-1); m++)
		{
			dayofy += dinmth[m];
			if ( m==2 ){
				if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0)
					dayofy += 1;
			}
		}
		dayofy += day;
	}

	///*  Convert day of year and year into week number and day of week */
	ttlday = 360;
	for (yr=1981; yr<=(year-1); yr++)
	{
		ttlday  += 365;
		if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 ==0)
		ttlday  += 1;
	}
	ttlday += dayofy;
	weekno  = ttlday/7;
	dayofw  = ttlday - 7 * weekno;

	gpst =  (second + minute*60 + hour*3600 + dayofw*86400)+utc.leapsec;
	week = weekno;
	leapsecond = utc.leapsec;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
#define SECONDS_IN_WEEK 604800.0
const static int leapsec_arr[][7]={
	{2012,7,1,9,0,0,16},
	{2009,1,1,0,0,0,15},
	{2006,1,1,0,0,0,14},
	{1999,1,1,0,0,0,13},
	{1997,7,1,0,0,0,12},
	{1996,1,1,0,0,0,11},
	{1994,7,1,0,0,0,10},
	{1993,7,1,0,0,0, 9},
	{1992,7,1,0,0,0, 8},
	{1991,1,1,0,0,0, 7},
	{1990,1,1,0,0,0, 6},
	{1988,1,1,0,0,0, 5},
	{1985,7,1,0,0,0, 4},
	{1983,7,1,0,0,0, 3},
	{1982,7,1,0,0,0, 2},
	{1981,7,1,0,0,0, 1}
};

int GetLeapSecondUTC(int year, int month, int mday,
	int hour /*= 0*/, int minute /*= 0*/, double second /*= 0*/)
{
	for(int i = 0; i < 16; i++){
		if (leapsec_arr[i][0] < year){
			return leapsec_arr[i][6];
			break;
		}
		else if (leapsec_arr[i][0] == year){
			if (leapsec_arr[i][1] < month){
				return leapsec_arr[i][6];
				break;
			}
			else if (leapsec_arr[i][1] == month){					
				if (leapsec_arr[i][2] <= mday){
					return leapsec_arr[i][6];
					break;
				}
				else{
					return (i == 0) ? 0 : leapsec_arr[i-1][6];
					break;
				}
			}
			else{
				return (i == 0) ? 0 : leapsec_arr[i-1][6];
				break;
			}
		}
		else
			continue;
	}

	return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
void GPStime2UTC(double gpst, int week, type_utc *utc)
{
	int i;
	int days, month, year;
	double offset, reftime, gpstime;
	int day,hour, min, tmp, leapsecond;

	gpstime = (week==0?gpst:gpst+week*SECONDS_IN_WEEK);
	for(i=0;i<15;i++){
		GetGPSTime(leapsec_arr[i][0], leapsec_arr[i][1], leapsec_arr[i][2],
			leapsec_arr[i][3], leapsec_arr[i][4], (double)leapsec_arr[i][5], &reftime);
		reftime += leapsec_arr[i][6];
		if(gpstime >= reftime){
			leapsecond = leapsec_arr[i][6];
			break;
		}
	}
	gpst = gpstime-(double)leapsecond;
	year=1980;
	int leapmon[12]={31, 29 ,31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  
	int noleapmon[12] = {31, 28, 31, 30, 31, 30,31, 31, 30, 31, 30, 31};
	if(week == 0){
		week = div((int)gpst,(3600*24*7)).quot;
		days=(week)*7+6;
	}else{
		days=(week)*7+6;
	}
	gpst = gpst-week*SECONDS_IN_WEEK;
	while(1){
		if(div(year,4).rem==0) {
			days-=366;
			year++;
			if(days<365) break;
		}
		else{
			days-=365;
			year++;
			if(div(year,4).rem==0 && days<366) break;
			else if(days<365) break;
		}
	}
	month=1;
	i=0;
	for(i=0;i<12;i++){
		if(div(year,4).rem==0){
			if(days<leapmon[i]) break;
			days-=leapmon[i];
			month++;
		}else{
			if(days<noleapmon[i]) break;
			days-=noleapmon[i];
			month++;
		}
	}
	 gpstime = gpst;
	double sec;
	day	= div((int)gpstime,(3600*24)).quot;
	tmp	= (int)gpstime-day*3600*24;
	hour	= div(tmp,3600).quot;
	tmp	= tmp-hour*3600;
	min	= div(tmp,60).quot;
	sec	= (double)(tmp-min*60);//+offset;
	if(div(year,4).quot == 0){
		if((days+day) > leapmon[month-1]){
			days = days+day-leapmon[month-1];
			month++;
		}else{
			days = days+day;
		}
		if(month>12) {
			year++;
		}
	}else{
		if((days+day) > leapmon[month-1]){
			days = days+day-leapmon[month-1];
			month++;
		}else{
			days = days+day;
		}
		if(month > 12) {
			year++;
		}
	}
	//year = year - 20;

	utc->year = year;
	utc->month = month;
	utc->day = day;
	utc->hour = hour;
	utc->minute = min;
	utc->sec = sec;
	utc->leapsec = leapsecond;
}

int GetGPSWeekNo(const double long_gpst)
{
	//int year = 0, month = 0, mday = 0, hour = 0, minute = 0;
	//double sec = 0;

	return (int)(long_gpst/604800.0);
	//lgpst2cgpst(long_gpst, 0, &year, &month, &mday, &hour, &minute, &sec);
	//return GetGPSWeekNo(year, month, mday, hour, minute, sec);
}
//int GetGPSWeekNo(int year, int month, int mday, int hour, int minute, double second)
//{
//	int		dayofw = 0, dayofy = 0, yr = 0, ttlday = 0, m = 0, weekno = 0;
//	int		dinmth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
//
//	/* Check limits of day, month and year */
//	//if (year < GPS_START_OF_YEAR || month < 1 || month > 12 || mday < 1 || mday > 31)
//	if (year < 1980 || month < 1 || month > 12 || mday < 1 || mday > 31)
//		weekno = 0;
//
//	/*  Convert day, month and year to day of year */
//	if (month == 1)
//		dayofy = mday;
//	else{		
//		for(m = 1; m <= (month-1); m++){
//			dayofy += dinmth[m-1];
//			if (m == 2){
//				if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0)// 윤년
//					dayofy += 1;
//			}
//		}
//		dayofy += mday;
//	}
//
//	/*  Convert day of year and year into week number and day of week */
//	ttlday = 360;// 365 - (GPS_START_OF_DAY - 1); GPST is zero at 1980.1.6. 00:00:00
//	//for (yr = GPS_START_OF_YEAR + 1; yr <= (year-1); yr++){
//	for (yr = 1981; yr <= (year-1); yr++){
//		ttlday  += 365;
//		if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 ==0)// 윤년
//			ttlday  += 1;
//	}
//	ttlday += dayofy;
//
//	return (int)(ttlday/7.0);	
//}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
void GPStime2UTC(double gpst, int week, int leapsecond, type_utc *utc)
{
	int i,j,k, year, month, day,hour, min,tmp, days;
	double sec;
	double gpstime, reftime, offset;
	//offset = mod2dec2(gpst);
	//gpst = gpst-offset;
	gpstime = (week==0?gpst:gpst+week*SECONDS_IN_WEEK);
	//TRACE("%f\n",gpstime);
	if(leapsecond == 0){
		for(i=0;i<15;i++){
			GetGPSTime(leapsec_arr[i][0], leapsec_arr[i][1], leapsec_arr[i][2],
				leapsec_arr[i][3], leapsec_arr[i][4], (double)leapsec_arr[i][5], &reftime);
			reftime += leapsec_arr[i][6];
			if(gpstime >= reftime){
				leapsecond = leapsec_arr[i][6];
				break;
			}
		}
	}

	gpst = gpstime-(double)leapsecond;
	year=1980;
	int leapmon[12]={31, 29 ,31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  
	int noleapmon[12] = {31, 28, 31, 30, 31, 30,31, 31, 30, 31, 30, 31};
	if(week == 0){
		week = div((int)floor(gpst),(3600*24*7)).quot;
		days=(week)*7+6;
	}else{
		days=(week)*7+6;
	}
	gpst = gpst-week*SECONDS_IN_WEEK;
	while(1){
		if(div(year,4).rem==0) {
			days-=366;
			year++;
			if(days<365) break;
		}
		else{
			days-=365;
			year++;
			if(div(year,4).rem==0 && days<366) break;
			else if(days<365) break;
		}
	}
	month=1;
	i=0;
	for(i=0;i<12;i++){
		if(div(year,4).rem==0){
			if(days<leapmon[i]) break;
			days-=leapmon[i];
			month++;
		}else{
			if(days<noleapmon[i]) break;
			days-=noleapmon[i];
			month++;
		}
	}
	 gpstime = gpst;
	 double dtmp;
	//day	= div((int)floor(gpstime),(3600*24)).quot;
	//tmp	= (int)floor(gpstime)-day*3600*24;
	//hour	= div((int)floor(tmp),3600).quot;
	//tmp	= tmp-hour*3600;
	//min	= div(tmp,60).quot;
	//sec	= (double)(tmp-min*60);//+offset;
	day	= div((int)floor(gpstime),(3600*24)).quot;
	dtmp	= gpstime-day*3600*24;
	hour	= div((int)floor(dtmp),3600).quot;
	dtmp	= dtmp-hour*3600;
	min	= div((int)floor(dtmp),60).quot;
	sec	= (dtmp-min*60);//+offset;
	if(div(year,4).quot == 0){
		if((days+day) > leapmon[month-1]){
			days = days+day-leapmon[month-1];
			month++;
		}else{
			days = days+day;
		}
		if(month>12) {
			year++;
		}
	}else{
		if((days+day) > leapmon[month-1]){
			days = days+day-leapmon[month-1];
			month++;
		}else{
			days = days+day;
		}
		if(month > 12) {
			year++;
		}
	}
	//year = year - 20;

	utc->year = year;
	utc->month = month;
	utc->day = days;
	utc->hour = hour;
	utc->minute = min;
	utc->sec = sec;
	utc->leapsec = leapsecond;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
int GetGPSTfrmUTC(int year,int month,int day, int hour,int minute, double second, double *gpstime, int &leapsec)
{
	int   dayofw,dayofy, yr, ttlday, m, weekno;
	static  int  dinmth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	int i;
	///* Check limits of day, month and year */
	if (year < 1981 || month < 1 || month > 12 || day < 1 || day > 31)
		weekno = 0;

	///*  Convert day, month and year to day of year */
	if (month == 1)
		dayofy = day;
	else
	{
		dayofy = 0;
		for (m=1; m<=(month-1); m++)
		{
			dayofy += dinmth[m];
			if ( m==2 )
			{
				if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0)
					dayofy += 1;
			}
		}
		dayofy += day;
	}

	///*  Convert day of year and year into week number and day of week */
	ttlday = 360;
	leapsec=0;
	for (yr=1981; yr<=(year-1); yr++)
	{
		ttlday  += 365;
		if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 ==0){
			ttlday  += 1;
		}
	}
	ttlday += dayofy;
	weekno  = ttlday/7;
	dayofw  = ttlday - 7 * weekno;

	for(i=0;i<15;i++){
		if((leapsec_arr[i][0]<=year)){
			if(leapsec_arr[i][1]<=month){
				if(leapsec_arr[i][2]<=day){
					leapsec = leapsec_arr[i][6];
					break;
				}
			}
		}
	}
	////*gpstime =  (hour * 3600 + minute * 60 + second);// + dayofw * 86400);
	*gpstime =  (second + minute*60 + hour*3600 + dayofw*86400 + weekno*SECONDS_IN_WEEK + leapsec);

	return weekno;
}






//***************************************************************//
//****************                            *******************//
//****************    LLH format Conversion   *******************//
//****************                            *******************//
//***************************************************************//

double NMEA0183ToDegree(double nmea)
{
	// nmea0183 : DDMM.MMMM(latitude) or DDDMM.MMMM(longitude), all positive.
	double dDeg = (double)(unsigned int)(nmea/100.0);
	double dMin = (nmea/100.0 - dDeg)*100.0;	

	return dDeg + dMin/60.0;
}

double DegreeToNMEA0183(double deg)
{
	// nmea0183 : DDMM.MMMM(latitude) or DDDMM.MMMM(longitude)
	double dDeg = (double)(unsigned int)deg;
	double dMin = (deg - dDeg)*60.0;

	return dDeg*100.0 + dMin;
}

double DMSToDegree(double dms)
{
	// DMS means : DD.MMSS	// ex) 37.3822 : degree : 37 , minute : 38, second : 22
	double dDeg = (double)(unsigned int)dms;
	double dMin = (double)(unsigned int)((dms - dDeg)*100.0);
	double dTmp = (double)(unsigned int)(dms*100.0);
	double dTmp2 = dms*100.0 - dTmp;	
	double dSecInt = (double)(unsigned int)(dTmp2*100.0);
	double dSecFlt = round(dTmp2*100.0 - dSecInt);
	double dSec = dSecInt + dSecFlt;

	return dDeg + dMin/60.0 + dSec/3600.0;
}

//// degree를 dms로 변환시 sec의 소수점을 날리기 때문에 값이 바뀌게 됩니다.
//// 즉, degree->dms 했을때 dms->deg로 변환하면 값이 0.xx 초 만큼의 값이 사라집니다.
double DegreeToDMS(double deg)
{
	// DMS means : DD.MMSS	// ex) 37.3822 : degree : 37 , minute : 38, second : 22

	double dDeg = (double)(unsigned int)deg;
	double dMin = (double)(unsigned int)((deg - dDeg)*60.0);
	double dSec = ((deg - dDeg)*60.0 - dMin)*60.0;
	double dTmp = dDeg + dMin/100.0 + dSec/10000.0;	
	return ((double)(unsigned int)(dTmp * 10000.0))/10000.0;
}





//***********************************************************************//
//********************                            ***********************//
//********************    Coordinate Conversion   ***********************//
//********************                            ***********************//
//***********************************************************************//

//// cdgpsins_hklee.cpp
Matrix Cxyz2enu(Matrix&  orgxyz)
{
	//XYZ2ENU  Convert from WGS-84 ECEF cartesian coordinates to 
	//               rectangular local-level-tangent ('East'-'North'-Up)
	//               coordinates.
	//
	//  enu  =  XYZ2ENU(xyz,orgxyz)  
	//
	//    INPUTS
	//  xyz(1)  =  ECEF x-coordinate in meters
	//  xyz(2)  =  ECEF y-coordinate in meters
	//  xyz(3)  =  ECEF z-coordinate in meters
	//
	//  orgxyz(1)  =  ECEF x-coordinate of local origin in meters
	//  orgxyz(2)  =  ECEF y-coordinate of local origin in meters
	//  orgxyz(3)  =  ECEF z-coordinate of local origin in meters
	//
	//    OUTPUTS
	//   Cxyz2enu

	//  Reference: Alfred Leick, GPS Satellite Surveying, 2nd ed.,
	//             Wiley-Interscience, John Wiley & Sons, 
	//             New York, 1995.
	//
	//  M. & S. Braasch 10-96
	//  Copyright (c) 1996 by GPSoft
	//  All Rights Reserved.

	Matrix tmpxyz, tmporg, difxyz, orgllh, R(3,3), enu;
	double phi, lam, sinphi, cosphi, sinlam, coslam;

	orgllh  =  xyz2llh(orgxyz);
	phi  =  orgllh(1,1);
	lam  =  orgllh(2,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	sinlam  =  sin(lam);
	coslam  =  cos(lam);

	R <<  -sinlam       <<  coslam          << 0
		<< -sinphi*coslam <<  -sinphi*sinlam  <<  cosphi
		<< cosphi*coslam  <<  cosphi*sinlam   << sinphi;

	return R;
}


//// cdgpsins_hklee.cpp
Matrix elazcal(double Xs,double Ys, double Zs, double Xu, double Yu, double Zu)
{
	//calculate elevation and azimuth
	//input   : satellite ECEF XYZ
	//		elazcal(Xs,Ys,Zs,Xu,Yu,Zu)
	//output : elevation/azimuth in radian
	//		elaz=[El Az]
	Matrix e, elaz;
	ColumnVector d;
	double x, y, z, p, R, s, El, Az;
	int k;

	e.ReSize(3,3); e = 0.0;
	x=Xu; y=Yu; z=Zu;
	p=sqrt(x*x+y*y);
	R=sqrt(x*x+y*y+z*z);

	e(1,1)=-(y/p);			e(1,2)=x/p;				e(1,3)=0.0;
	e(2,1)=-(x*z/(p*R));	e(2,2)=-(y*z/(p*R));	e(2,3)=p/R;
	e(3,1)=x/R;				e(3,2)=y/R;				e(3,3)=z/R;

	d.ReSize(3);

	for(k=1;k<=3; k++){
		d(k)=(Xs-Xu)*e(k,1)+(Ys-Yu)*e(k,2)+(Zs-Zu)*e(k,3);
	}

	s=d(3)/sqrt(d(1)*d(1)+d(2)*d(2)+d(3)*d(3));
	/////////////////////////////////	
	if (s==1.0){
		El=0.5*pi;
	}
	else{
		El=atan(s/sqrt(1.0-s*s));
	}
	/////////////////////////////////
	if ((d(2)==0.0)&(d(1)>0.0)){
		Az=0.5*pi;
	}
	else if((d(2)==0)&(d(1)<0.0)){
		Az=1.5*pi;
	}
	else{
		Az=atan(d(1)/d(2));
		if (d(2)<0.0){
			Az=Az+pi;
		}
		else if ((d(2)>0.0) & (d(1)<0.0)){
			Az=Az+2*pi;
		}
	}
	/////////////////////////////////	
	elaz.ReSize(2,1);
	elaz(1,1) = El;
	elaz(2,1) = Az;
	return elaz;
}


//// cdgpsins_hklee.cpp
Matrix enu2xyz(const Matrix& enu, const Matrix& orgxyz)
{
	//ENU2XYZ  Convert from rectangular local-level-tangent 
	//               ('East'-'North'-Up) coordinates to WGS-84 
	//               ECEF cartesian coordinates.
	//
	//  xyz  =  ENU2XYZ(enu,orgxyz)  
	//
	//  enu(1)  =  'East'-coordinate relative to local origin (meters)
	//  enu(2)  =  'North'-coordinate relative to local origin (meters)
	//  enu(3)  =  Up-coordinate relative to local origin (meters)
	//
	//  orgxyz(1)  =  ECEF x-coordinate of local origin in meters
	//  orgxyz(2)  =  ECEF y-coordinate of local origin in meters
	//  orgxyz(3)  =  ECEF z-coordinate of local origin in meters
	//
	//  xyz(1,1)  =  ECEF x-coordinate in meters
	//  xyz(2,1)  =  ECEF y-coordinate in meters
	//  xyz(3,1)  =  ECEF z-coordinate in meters
	//  Reference: Alfred Leick, GPS Satellite Surveying, 2nd ed.,
	//             Wiley-Interscience, John Wiley & Sons, 
	//             New York, 1995.
	//
	//  M. & S. Braasch 10-96
	//  Copyright (c) 1996 by GPSoft
	//  All Rights Reserved.

	Matrix tmpenu, tmpxyz, orgllh, R(3,3), difxyz, xyz;
	double phi, lam, sinphi, cosphi, sinlam, coslam; 

	if(enu.Nrows() < enu.Ncols() ){
		tmpenu = enu.t();
	}
	else{
		tmpenu = enu;
	}

	if(orgxyz.Nrows() < orgxyz.Ncols()){
		tmpxyz = orgxyz.t();
	}
	else{
		tmpxyz = orgxyz;
	}	

	orgllh  =  xyz2llh(tmpxyz);

	phi  =  orgllh(1,1);
	lam  =  orgllh(2,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	sinlam  =  sin(lam);
	coslam  =  cos(lam);

	R <<  -sinlam       <<  coslam          << 0
		<< -sinphi*coslam <<  -sinphi*sinlam  <<  cosphi
		<< cosphi*coslam  <<  cosphi*sinlam   << sinphi;

	difxyz  =  R.i()*tmpenu;
	xyz  =  tmpxyz + difxyz;

	return xyz;
}


//// cdgpsins_hklee.cpp
Matrix xyz2enu(const Matrix& xyz,const Matrix&  orgxyz)
{
	//XYZ2ENU  Convert from WGS-84 ECEF cartesian coordinates to 
	//               rectangular local-level-tangent ('East'-'North'-Up)
	//               coordinates.
	//
	//  enu  =  XYZ2ENU(xyz,orgxyz)  
	//
	//    INPUTS
	//  xyz(1)  =  ECEF x-coordinate in meters
	//  xyz(2)  =  ECEF y-coordinate in meters
	//  xyz(3)  =  ECEF z-coordinate in meters
	//
	//  orgxyz(1)  =  ECEF x-coordinate of local origin in meters
	//  orgxyz(2)  =  ECEF y-coordinate of local origin in meters
	//  orgxyz(3)  =  ECEF z-coordinate of local origin in meters
	//
	//    OUTPUTS
	//       enu:  Column vector
	//    enu(1,1)  =  'East'-coordinate relative to local origin (meters)
	//    enu(2,1)  =  'North'-coordinate relative to local origin (meters)
	//    enu(3,1)  =  Up-coordinate relative to local origin (meters)

	//  Reference: Alfred Leick, GPS Satellite Surveying, 2nd ed.,
	//             Wiley-Interscience, John Wiley & Sons, 
	//             New York, 1995.
	//
	//  M. & S. Braasch 10-96
	//  Copyright (c) 1996 by GPSoft
	//  All Rights Reserved.

	Matrix tmpxyz, tmporg, difxyz, orgllh, R(3,3), enu;
	double phi, lam, sinphi, cosphi, sinlam, coslam;

	tmpxyz  =  xyz;
	tmporg  =  orgxyz;
	if(tmpxyz.Nrows() !=  tmporg.Nrows() ) tmporg = tmporg.t(); 
	difxyz  =  tmpxyz - tmporg;
	if(difxyz.Nrows() < difxyz.Ncols() ) difxyz = difxyz.t();
	orgllh  =  xyz2llh(orgxyz);
	phi  =  orgllh(1,1);
	lam  =  orgllh(2,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	sinlam  =  sin(lam);
	coslam  =  cos(lam);

	R <<  -sinlam       <<  coslam          << 0
		<< -sinphi*coslam <<  -sinphi*sinlam  <<  cosphi
		<< cosphi*coslam  <<  cosphi*sinlam   << sinphi;

	enu  =  R*difxyz;
	return enu;
}


//// cdgpsins_hklee.cpp
Matrix llh2xyz(const Matrix& llh){
	//LLH2XYZ  Convert from latitude, longitude and height
	//         to ECEF cartesian coordinates.  WGS-84
	//
	//  xyz  =  LLH2XYZ(llh)  
	//
	//  llh(1)  =  latitude in radians
	//  llh(2)  =  longitude in radians
	//  llh(3)  =  height above ellipsoid in meters
	//
	//  xyz(1)  =  ECEF x-coordinate in meters
	//  xyz(2)  =  ECEF y-coordinate in meters
	//  xyz(3)  =  ECEF z-coordinate in meters

	//  Reference: Understanding GPS: Principles and Applications,
	//             Elliott D. Kaplan, Editor, Artech House Publishers,
	//             Boston, 1996.
	//
	//  M. & S. Braasch 10-96
	//  Copyright (c) 1996 by GPSoft
	//  All Rights Reserved.
	double phi, lamb, h, sinphi, cosphi, coslam,  sinlam, tan2phi, tmp, tmpden, tmp2;
	Matrix xyz(3,1);

	phi  =  llh(1,1);
	lamb  =  llh(2,1);
	h  =  llh(3,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	coslam  =  cos(lamb);
	sinlam  =  sin(lamb);
	tan2phi  =  pow(tan(phi),2.0);
	tmp  =  1 - epe*epe;
	tmpden  =  sqrt( 1 + tmp*tan2phi );
	xyz(1,1)  =  (epam*coslam)/tmpden + h*coslam*cosphi;
	xyz(2,1)  =  (epam*sinlam)/tmpden + h*sinlam*cosphi;
	tmp2  =  sqrt(1 - epe*epe*sinphi*sinphi);
	xyz(3,1)  =  (epam*tmp*sinphi)/tmp2 + h*sinphi;

	return xyz;
	phi  =  llh(1,1);
}

//// cdgpsins_hklee.cpp
Matrix xyz2llh(const Matrix& xyz){
	//XYZ2LLH	Convert from ECEF cartesian coordinates to 
	//               latitude, longitude and height.  WGS-84
	//
	//	llh  =  XYZ2LLH(xyz)	
	//
	//    INPUTS
	//	xyz(1)  =  ECEF x-coordinate in meters
	//	xyz(2)  =  ECEF y-coordinate in meters
	//	xyz(3)  =  ECEF z-coordinate in meters
	//
	//    OUTPUTS
	//	llh(1)  =  latitude in radians
	//	llh(2)  =  longitude in radians
	//	llh(3)  =  height above ellipsoid in meters

	//	Reference: Understanding GPS: Principles and Applications,
	//	           Elliott D. Kaplan, Editor, Artech House Publishers,
	//	           Boston, 1996.
	//
	//	M. & S. Braasch 10-96
	//	Copyright (c) 1996 by GPSoft
	//	All Rights Reserved.
	//

	/*
	double x, y, z;

	//double onethird; //added for the computation of s
	Matrix res(3,1);
	double tol = 1e-14;            // set a tight tolerance on the change in lat (rad)
	int max_iter = 15;			   // set maximum iterations
	int m = 1;					   // set first iteration
	double dlat= 1.0;			   // set the dlat value high for first iteration
	double e2,p,lat,lon, height, n,newlat;


	x  =  xyz(1,1);
	y  =  xyz(2,1);
	z  =  xyz(3,1);



	e2=(epam*epam-epbm*epbm)/(epam*epam);
	p=sqrt(x+y);
	lat=atan2(z,p*(1-e2));	   //initial estimate of the latitude

	lon=atan2(y,x);	// compute longitude
	// loop until the process has converged or maximum iterations are exceeded
	do
	{
	n = epam*epam/sqrt(epam*epam*cos(lat)*cos(lat) + epbm*epbm*sin(lat)*sin(lat));
	height = (p/cos(lat))-n;		// ellipsoidal height estimate 

	newlat=atan2(z,p*(1-e2*(n/(n+height))));
	dlat=fabs(lat-newlat);
	lat=newlat;
	m+=1;
	if(m>max_iter)
	{
	printf("Warning:Maximum iterations (%d) exceeded in ECEF2LLA!\n",max_iter);
	}
	}while(m<=max_iter && dlat>tol);

	res(1,1)  =  lat;
	res(2,1)  =  lon;
	res(3,1)  =  height;

	return res;
	*/

	double x, y, z, x2, y2, z2, b2, e2, ep, r, r2, E2, F, G, c, s, P, Q, ro;
	double tmp, U, V, zo, height, lat, temp, lon;
	//double onethird; //added for the computation of s
	Matrix res(3,1);

	x  =  xyz(1,1);
	y  =  xyz(2,1);
	z  =  xyz(3,1);

	x2  =  x*x;
	y2  =  y*y;
	z2  =  z*z;

	b2  =  epbm*epbm;
	e2  =  epe*epe;
	ep  =  epe*(epam/epbm);

	r  =  sqrt(x2+y2);
	r2  =  r*r;
	E2  =  epam*epam - epbm*epbm;
	F  =  54*b2*z2;
	G  =  r2 + (1-e2)*z2 - e2*E2;
	c  =  (e2*e2*F*r2)/(G*G*G);
	s = pow(1 + c + sqrt(c*c + 2*c),1.0/3.0); // note that pow(var1, 1/3) is false, and pow(var1, 1.0/3.0) is correct
	P  =  F / (3 * (s+1/s+1)*(s+1/s+1) * G*G);
	Q  =  sqrt(1+2*e2*e2*P);
	ro  =  -(P*e2*r)/(1+Q) + sqrt((epam*epam/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2);
	tmp  =  (r - e2*ro)*(r - e2*ro);
	U  =  sqrt( tmp + z2 );
	V  =  sqrt( tmp + (1-e2)*z2 );
	zo  =  (b2*z)/(epam*V);

	height  =  U*( 1 - b2/(epam*V) );

	if(r==0){
		lat = 0;
	}
	else{
		lat  =  atan( (z + ep*ep*zo)/r );
	}

	if(x == 0){
		temp  =  0; 
	}
	else{
		temp  =  atan(y/x);
	}

	if(x >= 0){	
		lon  =  temp;
	}
	else if( (x < 0) & (y >=  0) ){
		lon  =  pi + temp;
	}
	else{
		lon  =  temp - pi;
	}

	res(1,1)  =  lat;
	res(2,1)  =  lon;
	res(3,1)  =  height;

	return res;

}

//// cdgpsins_hklee.cpp
Matrix ned2xyz(const Matrix& ned, const Matrix& orgxyz){
	//  ned(1)  =  'East'-coordinate relative to local origin (meters)
	//  ned(2)  =  'North'-coordinate relative to local origin (meters)
	//  ned(3)  =  Up-coordinate relative to local origin (meters)
	//
	//  orgxyz(1)  =  ECEF x-coordinate of local origin in meters
	//  orgxyz(2)  =  ECEF y-coordinate of local origin in meters
	//  orgxyz(3)  =  ECEF z-coordinate of local origin in meters
	//
	//  xyz(1,1)  =  ECEF x-coordinate in meters
	//  xyz(2,1)  =  ECEF y-coordinate in meters
	//  xyz(3,1)  =  ECEF z-coordinate in meters
	Matrix tmpned, tmpxyz, orgllh, R(3,3), difxyz, xyz, iR(3,3);
	double phi, lam, sinphi, cosphi, sinlam, coslam; 

	if(ned.Nrows() < ned.Ncols() ){
		tmpned = ned.t();
	}
	else{
		tmpned = ned;
	}

	if(orgxyz.Nrows() < orgxyz.Ncols()){
		tmpxyz = orgxyz.t();
	}
	else{
		tmpxyz = orgxyz;
	}

	orgllh  =  xyz2llh(tmpxyz);

	phi  =  orgllh(1,1);
	lam  =  orgllh(2,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	sinlam  =  sin(lam);
	coslam  =  cos(lam);

	R << -sinphi*coslam <<  -sinphi*sinlam  <<  cosphi         
		<<  -sinlam       <<  coslam          << 0               
		<< -cosphi*coslam  <<  -cosphi*sinlam   << -sinphi;   

	iR = R.i();

	difxyz  =  iR*tmpned;
	xyz  =  tmpxyz + difxyz;

	return xyz;
}


//// cdgpsins_hklee.cpp
Matrix xyz2ned(const Matrix& xyz, const Matrix&  orgxyz){
	//  xyz(1)  =  ECEF x-coordinate in meters
	//  xyz(2)  =  ECEF y-coordinate in meters
	//  xyz(3)  =  ECEF z-coordinate in meters
	//
	//  orgxyz(1)  =  ECEF x-coordinate of local origin in meters
	//  orgxyz(2)  =  ECEF y-coordinate of local origin in meters
	//  orgxyz(3)  =  ECEF z-coordinate of local origin in meters
	//
	//    OUTPUTS
	//       enu:  Column vector
	//    ned(1,1)  =  'East'-coordinate relative to local origin (meters)
	//    ned(2,1)  =  'North'-coordinate relative to local origin (meters)
	//    ned(3,1)  =  Up-coordinate relative to local origin (meters)

	Matrix tmpxyz, tmporg, difxyz, orgllh, R(3,3), ned;
	double phi, lam, sinphi, cosphi, sinlam, coslam;

	tmpxyz  =  xyz;
	tmporg  =  orgxyz;
	if(tmpxyz.Nrows() !=  tmporg.Nrows() ) tmporg = tmporg.t(); 
	difxyz  =  tmpxyz - tmporg;
	if(difxyz.Nrows() < difxyz.Ncols() ) difxyz = difxyz.t();
	orgllh  =  xyz2llh(orgxyz);
	phi  =  orgllh(1,1);
	lam  =  orgllh(2,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	sinlam  =  sin(lam);
	coslam  =  cos(lam);

	R << -sinphi*coslam <<  -sinphi*sinlam  <<  cosphi         
		<<  -sinlam       <<  coslam          << 0               
		<< -cosphi*coslam  <<  -cosphi*sinlam   << -sinphi;      

	ned  =  R*difxyz;
	return ned;
}

//// cdgpsins_hklee.cpp
Matrix nedv2xyzv(const Matrix& ned, const Matrix& orgxyz){
	// transform ned velocity to ECEF ned velocity
	// coriolis term is neglected, vehicle velocity is assumed small
	//
	//  ned(1)  =  'North'-coordinate relative to local origin (meters)
	//  ned(2)  =  'East'-coordinate relative to local origin (meters)
	//  ned(3)  =  Down-coordinate relative to local origin (meters)
	//
	//  orgxyz(1)  =  ECEF x-coordinate of local origin in meters
	//  orgxyz(2)  =  ECEF y-coordinate of local origin in meters
	//  orgxyz(3)  =  ECEF z-coordinate of local origin in meters
	//
	//  xyz(1,1)  =  ECEF x-coordinate in meters
	//  xyz(2,1)  =  ECEF y-coordinate in meters
	//  xyz(3,1)  =  ECEF z-coordinate in meters
	Matrix tmpned, tmpxyz, orgllh, R(3,3), difxyz, xyz, iR(3,3);
	double phi, lam, sinphi, cosphi, sinlam, coslam; 

	if(ned.Nrows() < ned.Ncols() ){
		tmpned = ned.t();
	}
	else{
		tmpned = ned;
	}

	if(orgxyz.Nrows() < orgxyz.Ncols()){
		tmpxyz = orgxyz.t();
	}
	else{
		tmpxyz = orgxyz;
	}

	orgllh  =  xyz2llh(tmpxyz);

	phi  =  orgllh(1,1);
	lam  =  orgllh(2,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	sinlam  =  sin(lam);
	coslam  =  cos(lam);

	R << -sinphi*coslam <<  -sinphi*sinlam  <<  cosphi         
		<<  -sinlam       <<  coslam          << 0               
		<< -cosphi*coslam  <<  -cosphi*sinlam   << -sinphi;   

	iR = R.i();

	difxyz  =  iR*tmpned;
	xyz  =  difxyz;

	return xyz;
}


//// cdgpsins_hklee.cpp
Matrix geocenllh2xyz(const Matrix& llh){
	//LLH2XYZ  Convert from latitude, longitude and height
	//         to ECEF cartesian coordinates.  WGS-84
	//
	//  xyz  =  LLH2XYZ(llh)  
	//
	//  llh(1)  =  geocentric latitude in radians
	//  llh(2)  =  geocentric longitude in radians
	//  llh(3)  =  geocentric height above mean radius in meters
	//
	//  xyz(1)  =  ECEF x-coordinate in meters
	//  xyz(2)  =  ECEF y-coordinate in meters
	//  xyz(3)  =  ECEF z-coordinate in meters

	double phi, lamb, h, sinphi, cosphi, coslam,  sinlam, tan2phi/*, tmp, tmpden, tmp2*/, R0;
	Matrix xyz(3,1);

	R0 = 6371007.2;

	phi  =  llh(1,1);
	lamb  =  llh(2,1);
	h  =  llh(3,1);
	sinphi  =  sin(phi);
	cosphi  =  cos(phi);
	coslam  =  cos(lamb);
	sinlam  =  sin(lamb);
	tan2phi  =  pow(tan(phi),2.0);
	xyz(1,1)  =  (R0+h)*cosphi*coslam; 
	xyz(2,1)  =  (R0+h)*cosphi*sinlam; 
	xyz(3,1)  =  (R0+h)*sinphi;

	return xyz;
}


//// cdgpsins_hklee.cpp
Matrix xyz2geocenllh(const Matrix& xyz){
	//XYZ2LLH	Convert from ECEF cartesian coordinates to 
	//               latitude, longitude and height.  WGS-84
	//
	//	llh  =  XYZ2LLH(xyz)	
	//
	//    INPUTS
	//	xyz(1)  =  ECEF x-coordinate in meters
	//	xyz(2)  =  ECEF y-coordinate in meters
	//	xyz(3)  =  ECEF z-coordinate in meters
	//
	//    OUTPUTS
	//  llh(1)  =  geocentric latitude in radians
	//  llh(2)  =  geocentric longitude in radians
	//  llh(3)  =  geocentric height above mean radius in meters



	double x, y, z, x2, y2, z2, /*b2, e2, ep,*/ r, r2;//, E2, F, G, c, /*s, P, Q,*/ ro;
	double /*tmp, V, zo,*/ U, height, lat, temp, lon, R0;
	//double onethird; //added for the computation of s
	Matrix res(3,1);

	R0 = epam;

	x  =  xyz(1,1);
	y  =  xyz(2,1);
	z  =  xyz(3,1);

	x2  =  x*x;
	y2  =  y*y;
	z2  =  z*z;

	r  =  sqrt(x2+y2);
	r2  =  r*r;
	U  =  sqrt( x2 + y2 + z2 );

	height  =  U - R0;

	if(r==0){
		lat = 0;
	}
	else{
		lat  =  atan( z/r );
	}

	if(x == 0){
		temp  =  0; 
	}
	else{
		temp  =  atan(y/x);
	}

	if(x >= 0){	
		lon  =  temp;
	}
	else if( (x < 0) & (y >=  0) ){
		lon  =  pi + temp;
	}
	else{
		lon  =  temp - pi;
	}

	res(1,1)  =  lat;
	res(2,1)  =  lon;
	res(3,1)  =  height;

	return res;
}



//***********************************************************************//
//********************                            ***********************//
//********************    etc                     ***********************//
//********************                            ***********************//
//***********************************************************************//

//// cdgpsins_hklee.cpp
double norm(const Matrix& M){
	// norm of a row or colum matrix
	// whose dimension is (nr,1) or (1, nc)
	// returns zero or positive if success
	// returns -1 if fail
	int nr, nc;
	Matrix res(1,1);
	double nm;

	nr = M.Nrows();
	nc = M.Ncols();

	if(nc == 1){
		res = M.t()*M;
		nm=sqrt(res(1,1));
		return nm;
	}
	else if(nr == 1){
		res = M*M.t();
		nm=sqrt(res(1,1));
		return nm;
	}
	else{
		//printf("Error : Matrix Dimension is not correct in computing norm()");
		return -1;
	}
}

//// cdgpsins_hklee.cpp
double round(const double val){
	double a, b, c;

	if(val>=0){
		a = val + 0.5;
		b = floor(a);
		return b;
	}
	else{
		a = -val + 0.5;
		b = floor(a);
		c = -b;
		return c;
	}
}


//// print out matrix
void PrintT(const Matrix &m)
{
	TRACE("\nMatrix type: ", m.Type().Value());
	TRACE("(%d, %d)\n\n", m.Nrows(), m.Ncols());
	if (m.IsZero()){
		TRACE("All elements are zero\n");	return;
	}

	int nr = m.Nrows();
	int nc = m.Ncols();

	for (int i = 1; i <= nr; ++i){
		for (int j = 1; j <= nc; ++j){
			//if (fabs(m(i,j))>1e10)	TRACE("%.7f\t",m(i,j));
			//else					TRACE("%.7f\t",m(i,j));
			TRACE("%.7f\t", (double)m(i,j));
		}
		TRACE("; %d\n", i);
	}
}

void fPrintT(FILE* fp, const Matrix &m)
{
	if (fp == NULL){
		TRACE("\r\n\t file pointer == null\r\n");	return;		
	}
	else{
		fprintf(fp, "\nMatrix type: ", m.Type().Value());
		fprintf(fp, "(%d, %d)\n\n", m.Nrows(), m.Ncols());
		if (m.IsZero()){
			fprintf(fp, "All elements are zero\n");	return;
		}

		int nr = m.Nrows();
		int nc = m.Ncols();

		for (int i = 1; i <= nr; ++i){
			for (int j = 1; j <= nc; ++j){
				//if (fabs(m(i,j))>1e10)	TRACE("%.7f\t",m(i,j));
				//else					TRACE("%.7f\t",m(i,j));
				fprintf(fp, "%.7f\t", (double)m(i,j));
			}
			fprintf(fp, "; %d\n", i);
		}
	}
}