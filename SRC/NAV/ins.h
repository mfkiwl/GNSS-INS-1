
#pragma once
//#include "stdafx.h"

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH                    // include.h will get math fns
// newmatap.h will get include.h
#include "../NEWMAT10D/newmatap.h"                // need Matrix applications
#include "../NEWMAT10D/newmatio.h"                // need Matrix output routines
#include "../NEWMAT10D/precisio.h"				         // needed for the def. of eps
#include "../NEWMAT10D/newmat.h"

#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif

#include "Structure_type.h"


#define r2d				57.29577951308238	//	radian to degree
#define d2r				0.01745329251994	//	degree to radian
//#define IMU_SMPL_RATE		20	// Hz
#define IMU_SMPL_PERIOD		0.01	// sec
#define WGS84OE		7.2921151467E-5	//	in ICD-GPS-200
//===Choice 1: WGS-84 parameters===//
#define epam			6378137.0000	//	earth semimajor axis in meters
#define epbm			6356752.3142	//	earth semiminor axis in meters
#define epe				8.181919092890692e-002 // earth eccentricity, i.e. sqrt(1-(epbm/epam)*(epbm/epam));
#define gm				9.780327		//	earth gravity constant in meters
#define gf				32.2			//	# gravity G (in feet)
#define erot			7.292115e-5		//	# earth rotatio rate (in rad/sec)


#define ZV_SF_OFS				2.0			//	zero velocity condition threshhold for specific force magnitude offset in ft/sec^2
#define SECONDS_IN_WEEK			604800.0
//#define RE		2.097360e7
//#define EC		0.0066743123
//#define GG1 	0.170141635
//#define GG2		-3.0877e-6
//#define GG3		4.3e-9
//#define GG4		2.194559929e-13
#define f2m		0.30479951			//	conversion from feet to m
#define pi		3.14159265358979 

int		read_IMU(type_raw_imu *im,  FILE *fimu);
int		nav_type_init_tight(type_avr_imu *a,type_hspd_sdins *h, type_mspd_sdins *m,type_at_epoch *e);
int		course_align(type_avr_imu *avr, type_hspd_sdins *h, type_mspd_sdins *m, type_gps_sol *gpssol);
void	init_nav_by_lgps(type_gps_sol& igpsl, type_mspd_sdins& m, type_hspd_sdins& h);
int		tframe_covar_tight(type_at_epoch *e);
//int		brm_avr_imu(type_hspd_sdins *h, type_raw_imu **raw, type_avr_imu *avr, int idx, int flag);
int		brm_avr_imu(type_hspd_sdins *h, type_raw_imu *raw, type_avr_imu *avr, int idx, int flag);
int		do_hspd_sdins_in_rx_seq(type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *e, type_avr_imu *s);
int		do_mspd_sdins(type_hspd_sdins *h, type_mspd_sdins *m,type_at_epoch *pps);
int		tframe_do_at_gps_tight_rx_in_rx_seq(type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *e, type_avr_imu *s, type_gps_sol *tgps, int fava_gps);
void	save_shm_for_filter(type_avr_imu *s, type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *e);
int		tframe_do_propa_tight_newmat(type_at_epoch *e, type_mspd_sdins *m);
int		tframe_do_mupdate_tight_newmat(type_hspd_sdins *h, type_at_epoch *e, type_gps_sol *tgps);
int		tframe_compen_hspd_sdins(type_hspd_sdins *h, type_at_epoch *e, int flag);
int		tframe_compen_mspd_sdins(type_mspd_sdins *m, type_at_epoch *e, int flag);

//double	round		(const double val);
//void	PrintT		(const Matrix &m);
//Matrix	llh2xyz		(const Matrix& llh);


int		brm_avr_imu_pp(type_hspd_sdins *h, type_raw_imu **raw, type_avr_imu *avr, int idx, int flag);