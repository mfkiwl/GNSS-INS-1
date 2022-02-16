#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <malloc.h>

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH                    // include.h will get math fns
// newmatap.h will get include.h

#include "Matrix/newmatap.h"                // need Matrix applications
#include "Matrix/newmatio.h"                // need Matrix output routines
#include "Matrix/precisio.h"				         // needed for the def. of eps
#include "Matrix/newmat.h"

#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif

#include "ins.h"

#define MAX_STRUCTSIZE	172800
#define MAX_LINE		256
#define MAX_LEAPSEC		17				//	maximum number of leapsecond array
//////////////////////////////////////////////////////
FILE *fp_input;
char *input_file;
int gps_read_cnt;
int rmc_f, gga_f, vtg_f;
int flag_gps_new, flag_init, ini_imu_flag;

type_gps_sol			gps_sol;			// GPS 관련 구조체
type_gps_sol			pre_gps_sol;		// GPS 관련 구조체
type_raw_imu			**pprawimu;				// IMU/SPD 관련 구조체

type_hspd_sdins			h;					// 자세 관련 구조체
type_mspd_sdins			m;					// 속도 관련 구조체
type_at_epoch			pps;				// 필터 관련 구조체
type_avr_imu			avr;				// 바이어스가 제거된 측정치

int flag_gps, flag_ins;

int Parser			(char* stmp, char *tmp);
int	GetGPSTfrmUTC	(int year,int month,int day, int hour,int minute,double second,double *gpstime, int &leapsec);