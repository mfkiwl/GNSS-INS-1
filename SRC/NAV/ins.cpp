#pragma once


#pragma warning(disable:4101)
#pragma warning(disable:4474)

#include "ins.h"
#include <math.h>
#include "../SUBFUNC/SUBFUNC.h"




int		read_IMU			(type_raw_imu *im,  FILE *fimu)
{
	int i, res;

	double temp[12];

	//for( i=0; i<12;i++){
	for (i = 0; i < 11; i++) {
		res = fscanf(fimu,"%lf",&(temp[i]) );
	}

	if ((res == 0) || (res == EOF)) {
		return -1;
	}

	if (temp[1] >= (int)(1.0 / IMU_SMPL_PERIOD))
	{
		return 0;
	}

	//im->cnt_frm_epoch = (temp[1]-1)/100.0;//+1;
	//im->pgps_time = im->gps_time;
	//im->gps_time = round(temp[0])  + im->cnt_frm_epoch;

	im->cnt_frm_epoch = (temp[1]) / (1.0 / IMU_SMPL_PERIOD);//+1;
	im->pgps_time = im->gps_time;
	im->gps_time = round(temp[0]) + im->cnt_frm_epoch;


	/*im->fb[0] = temp[5];
	im->fb[1] = -temp[6];
	im->fb[2] = -temp[7];

	im->rbib[0] = temp[8];
	im->rbib[1] = -temp[9];
	im->rbib[2] = -temp[10];*/

	im->fb[0] = temp[5];// / 1000 * 9.8;
	im->fb[1] = temp[6];// / 1000 * 9.8;
	im->fb[2] = temp[7];// / 1000 * 9.8;

	im->rbib[0] = temp[2];// * d2r;
	im->rbib[1] = temp[3];// * d2r;
	im->rbib[2] = temp[4];// * d2r;



	//im->fb[0] = im->fb[0] / (1.0 / IMU_SMPL_PERIOD);
	//im->fb[1] = im->fb[1] / (1.0 / IMU_SMPL_PERIOD);
	//im->fb[2] = im->fb[2] / (1.0 / IMU_SMPL_PERIOD);

	//im->rbib[0] = im->rbib[0] / (1.0 / IMU_SMPL_PERIOD);
	//im->rbib[1] = im->rbib[1] / (1.0 / IMU_SMPL_PERIOD);
	//im->rbib[2] = im->rbib[2] / (1.0 / IMU_SMPL_PERIOD);





	//im->fb[0] = temp[5] / 1000 * 9.8;
	//im->fb[1] = temp[6] / 1000 * 9.8;
	//im->fb[2] = temp[7] / 1000 * 9.8;

	//im->rbib[0] = temp[2] * d2r;
	//im->rbib[1] = temp[3] * d2r;
	//im->rbib[2] = temp[4] * d2r;


	//double sfa = 100.0;
	//double sfg = 100.0;

	//im->fb[0] = (temp[5] / sfa) / 1000 * 9.8;
	//im->fb[1] = (temp[6] / sfa) / 1000 * 9.8;
	//im->fb[2] = (temp[7] / sfa) / 1000 * 9.8;

	//im->rbib[0] = (temp[2] / sfg) * d2r;
	//im->rbib[1] = (temp[3] / sfg) * d2r;
	//im->rbib[2] = (temp[4] / sfg) * d2r;





	//im->fb[0] = (temp[5] / 65536.0 / 100.0) / 1000 * 9.8;
	//im->fb[1] = (temp[6] / 65536.0 / 100.0) / 1000 * 9.8;
	//im->fb[2] = (temp[7] / 65536.0 / 100.0) / 1000 * 9.8;

	//im->rbib[0] = (temp[2] / 65536.0 / 100.0) * d2r;
	//im->rbib[1] = (temp[3] / 65536.0 / 100.0) * d2r;
	//im->rbib[2] = (temp[4] / 65536.0 / 100.0) * d2r;





	//im->fb[0] = (temp[5] / 100.0) / 1000 * 9.8;
	//im->fb[1] = (temp[6] / 100.0) / 1000 * 9.8;
	//im->fb[2] = (temp[7] / 100.0) / 1000 * 9.8;

	//im->rbib[0] = temp[2] * d2r;
	//im->rbib[1] = temp[3] * d2r;
	//im->rbib[2] = temp[4] * d2r;






	//if (sqrt(im->rbib[2]*im->rbib[2]) <= 0.0607)
	//if (sqrt(im->rbib[2]*im->rbib[2]) <= 0.1742)
	//{
	//	im->fb[1] = 0;
	//	//im->fb[2] = 0;
	//	im->rbib[1] = 0;
	//	im->rbib[2] = 0;
	//}

	return 1;
}

int		nav_type_init_tight(type_avr_imu *a, type_hspd_sdins *h, type_mspd_sdins *m,type_at_epoch *e)
{
	// returns 1 if success
	// returns 0 if fail
	//---------------------
	// Here, two functions, each using two text data file names, i.e.
	// 1. get_value_frm_file("timing_init_gpsins.m","search string",&tmp);
	// 2. get_value_frm_file("nav_init_gpsins.m","search string",&tmp);
	// are used for the initial value input source
	int i,j,k,n;
	//-----------type_avr_imu *a---------------------------------------//
	// bias
	for(i=0;i<3;i++){
		a->ba[i]=0;
		a->bg[i]=0;
	}

	//a->msamp = 2;
	a->msamp = 10;
	e->delay = 0;//0.05;	// (1/Hz)

	// previously sampled data storage
	for(i=0;i<(a->msamp);i++){
		for(j=0;j<3;j++){
			a->p_fb[i][j]=0;
			a->p_rbib[i][j]=0;
		}
	} 
	a->sts = 1;

	for(i=0;i<3;i++){
		h->acc_fn[i]=0;
		h->dang[i]=0;
		h->ba[i]=0;
		h->bg[i]=0;
	}

	//m->lat = 37.4512 * d2r;
	//m->lon = 126.9523 * d2r;
	//m->hgt = 574.8146;

	m->lat = 37.367928000000049 * d2r;
	m->lon = 126.71827399999992 * d2r;
	m->hgt = 113.94753519140382;

	m->v[0] = 0.0;
	m->v[1] = 0.0;
	m->v[2] = 0.0;

	// dllh
	m->dlat = 0;	m->dlon = 0;	m->dhgt = 0;
	// dv, ba, bg
	for(i=0;i<3;i++){
		m->dv[i]=0;
		m->ba[i]=0;
		m->bg[i]=0;
	}


	m->er = erot;

	m->clat = cos(m->lat);
	m->slat = sin(m->lat);
	m->slat2 = m->slat * m->slat;

	double oes, g0;


	oes = sqrt(1-epe*epe*m->slat2);
	m->rm = epam*(1-epe*epe)/(oes*oes*oes);
	m->rt = epam/oes;
	g0 = gm*(1+0.0053024*m->slat2 - 0.0000058*sin(2*m->lat)*sin(2*m->lat)); // latitude correction of local gravity 
	m->g = g0 - (3.0877e-6 - 0.0044e-6*m->slat2)*m->hgt + 0.072e-12*(m->hgt*m->hgt); // height correction of local gravity 



	//pgps_time
	m->pgps_time = 0;
	m->sts = 1;
	for(i=0;i<3;i++){
		m->rnin[i]=0;
	}

	e->p_sts = 0;

	e->kvdt = 0.0;
	e->kpdt = 0.0;

	e->cotend = 0.0;
	e->fitend = 0.0;
	e->natend = 0.0;

	// rstcnt_to_p_up; // reset value : (int) kpdt/kvdt 
	e->rstcnt_to_p_up = (int) ((e->kpdt)/(e->kvdt));
	// rmcnt_to_p_up;	// remaining count to next position-aided filter update
	e->rmcnt_to_p_up = e->rstcnt_to_p_up;
	// gps_time_last_propa; // time when last Kalman filter meas. update was achieved.
	e->gps_time_last_propa = 0;
	// gps_time_last_meas_data; // time of the last meas. data for Kalman meas. update
	e->gps_time_last_meas_data = -1;
	// m_up_type; // required measure update type
	// 0 for zero velocity update(when vehicle is not in movement)
	// 1 for gps vel. update only
	// 2 for gps pos. update only
	// 3 for gps pos & vel combined update
	// 4 for gps pos & zero vel combined update
	e->m_up_type = 2;
	// n // matrix order of the Kalman filter & error covar. mtx.

	e->n = 15;
	e->vel_scale = 0;
	//
	n= (e->n) -1; 

	for(j=0;j<=n;j++){
		for(k=0;k<=n;k++){
			e->P[j][k]=0.0;
		}
		e->Q[j]=0.0;
	}

	e->zvelstd = 0.01;
	e->gpsvelstd = 7;

	e->sts = 1;
	// msd of e
	//-----------type_mspd_sdins e->msd---------------------------------------//
	//dt
	m->dt = 0.01;	// add temp
	e->dt = m->dt;
	//integ_timing_reference
	e->integ_timing_reference = m->integ_timing_reference;
	// lat, lon, hgt

	e->lat = m->lat;
	e->lon = m->lon;
	e->hgt = m->hgt;

	// v
	e->v[0] = m->v[0];
	e->v[1] = m->v[1];
	e->v[2] = m->v[2];
	// dllh
	e->dlat = 0;	e->dlon = 0;	e->dhgt = 0;
	// dv, ba, bg
	for(i=0;i<3;i++){
		e->dv[i]=0;
		e->ba[i]=0;
		e->bg[i]=0;
	}
	// g
	e->g = m->g;
	// er
	e->er = m->er;
	//pgps_time
	e->pgps_time = 0;
	e->sts = 1;

	// codes added by debug 3.19/2000
	// These codes are needed especially when propa. precedes m_update
	e->fn[0] = 0;
	e->fn[1] = 0;	
	e->fn[2] = e->g;
	e->rnie[0] = e->er * e->clat;
	e->rnie[1] = 0;	
	e->rnie[2] = - e->er * e->slat;
	e->rnen[0] = e->dlon * e->clat;
	e->rnen[1] = -e->dlat;
	e->rnen[2] = -e->dlon * e->slat;
	e->rnin[0] = e->rnie[0] + e->rnen[0];
	e->rnin[1] = e->rnie[1] + e->rnen[1];
	e->rnin[2] = e->rnie[2] + e->rnen[2];
	e->cor[0] = 2 * e->rnie[0] + e->rnen[0];
	e->cor[1] = 2 * e->rnie[1] + e->rnen[1];
	e->cor[2] = 2 * e->rnie[2] + e->rnen[2];	
	e->slat = sin(e->lat);
	e->clat = cos(e->lat);
	e->slat2 = (e->slat)*(e->slat);
	oes = sqrt(1-epe*epe*e->slat2);
	e->rm = epam*(1-epe*epe)/(oes*oes*oes);
	e->rt = epam/oes;
	e->rmh = e->rm + e->hgt;
	e->rth = e->rt + e->hgt;
	e->g = m->g; // height correction of local gravity
	e->numcr = 0;

	// codes added by debug 12.27/2000
	// These codes are needed for tightly-coupled configuration
	e->clockbias = 0; 
	e->clockbrate = 0; 

	// optional initialization for feedforward federated filter architecture
	for(i=0;i<=n;i++)	e->x[i] = 0.0;
	//
	h->ce_time=-1.0;
	m->ce_time=-1.0;
	e->ce_time=-1.0;

	e->lmdist[0] = 0;
	e->lmdist[1] = 0;
	e->lmdist[2] = 0.2;


	return 1;	
}

int		course_align(type_avr_imu *avr, type_hspd_sdins *h, type_mspd_sdins *m, type_gps_sol *gpssol){

	// The course alignment algorithm
	//
	// returns 1 if success
	// returns 0 if fail
	//
	// note that 
	// h and m are used for writing the result of course align
	// 

	int j;
	double  axa,aya,aza,rxa,rya,rza;    // mean value of sensor data
	double  spci, cpci;
	double  cpi,spi,cth,sth,cps,sps;
	double  r0, r1, r2, r3, r4, r5, r6, r7, r8, r9;
	double  tem1;
	double  roll, pitch, yaw;

	//*** derive attitude
	//** compute the average value of sensor data/
	axa = avr->fb[0];//axsum / update_num1;     // accel mean value
	aya = avr->fb[1];//aysum / update_num1;
	aza = avr->fb[2];//azsum / update_num1;
	rxa = avr->rbib[0];//rxsum / update_num1;     // gyro mean value
	rya = avr->rbib[1];//rysum / update_num1;
	rza = avr->rbib[2];//rzsum / update_num1;


	//** compute theta(pitch) angle
	tem1 = sqrt( aya*aya + aza*aza);
	pitch = atan( axa / tem1);	

	//* compute phi(roll) angle
	spci = -aya / tem1;
	cpci = -aza / tem1;
	roll = atan2(spci,cpci);
	//roll = atan(aya/aza);
	
	yaw = gpssol->hd*d2r; 

	//** compute the quaternions
	cpi = cos (roll / 2);
	spi = sin (roll / 2);
	cth = cos (pitch / 2);
	sth = sin (pitch / 2);
	cps = cos ((yaw)/ 2);
	sps = sin ((yaw)/ 2);

	h->q[0] =  cpi * cth * cps + spi * sth * sps;
	h->q[1] =  spi * cth * cps - cpi * sth * sps;
	h->q[2] =  cpi * sth * cps + spi * cth * sps;
	h->q[3] = -spi * sth * cps + cpi * cth * sps;

	/* compute body to navigation transformation matrix */
	r0 = h->q[0] * h->q[0];
	r1 = h->q[1] * h->q[1];
	r2 = h->q[2] * h->q[2];
	r3 = h->q[3] * h->q[3];
	r4 = h->q[0] * h->q[1];
	r5 = h->q[0] * h->q[2];
	r6 = h->q[0] * h->q[3];
	r7 = h->q[1] * h->q[2];
	r8 = h->q[1] * h->q[3];
	r9 = h->q[2] * h->q[3];

	h->c[0] = r0 + r1 - r2 - r3;
	h->c[1] = 2 * (r7 - r6);
	h->c[2] = 2 * (r5 + r8);
	h->c[3] = 2 * (r6 + r7);
	h->c[4] = r0 - r1 + r2 - r3;
	h->c[5] = 2 * (r9 - r4);
	h->c[6] = 2 * (r8 - r5);
	h->c[7] = 2 * (r9 + r4);
	h->c[8] = r0 - r1 - r2 + r3;

	for(j=0;j<4;j++) m->q[j] = h->q[j];
	for(j=0;j<9;j++) m->c[j] = h->c[j];

	
	avr->ca_roll = roll * r2d;
	avr->ca_pitch = pitch * r2d;
	avr->ca_yaw = yaw * r2d;


	printf("\n=======================\n");
	printf("Course align result :\n");
	printf("roll(deg) = %.10f\n",roll*r2d);
	printf("pitch(deg) = %.10f\n",pitch*r2d);
	printf("heading(deg) = %f\n",yaw*r2d);
	printf("=======================\n");

	return 1;
}

void	init_nav_by_lgps(type_gps_sol& igpsl, type_mspd_sdins& m, type_hspd_sdins& h)
{
	int i;
	double gpshd, oes;
	// initialize INS data structure 
	gpshd = igpsl.hd;

	// medium speed
	m.lat = igpsl.lat*d2r;// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<INIT
	m.lon = igpsl.lon*d2r;// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<INIT
	m.hgt = igpsl.hgt;;// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<INIT
	m.v[0] = igpsl.vel_horz*cos(gpshd*d2r);
	m.v[1] = igpsl.vel_horz*sin(gpshd*d2r);
	//m.v[2] = -igpsl.vel_vert;
	m.v[2] = 0;
	m.smlat = 0;
	m.smlon = 0;
	m.lat0 = m.lat;
	m.lon0 = m.lon;
	m.dt = 	IMU_SMPL_PERIOD;
	m.integ_timing_reference = 0;
	m.er = WGS84OE;

	oes = sqrt(1-epe*epe*m.slat2);
	m.rm = epam*(1-epe*epe)/(oes*oes*oes);
	m.rt = epam/oes;
	m.rmh = m.rm + m.hgt;
	m.rth = m.rt + m.hgt;

	m.g = gm*(1+0.0053024*sin(m.lat)*sin(m.lat)); // latitude correction of local gravity 


	m.dlat = m.v[0] / (m.rmh);
	m.dlon = m.v[1] / ((m.rth) * cos(m.lat));
	m.dhgt = - m.v[2];
	m.rnie[0] = m.er * cos(m.lat);
	m.rnie[1] = 0; 
	m.rnie[2] = - m.er * sin(m.lat);
	m.rnen[0] = m.dlon * cos(m.lat);
	m.rnen[1] = - m.dlat;
	m.rnen[2] = - m.dlon * sin(m.lat);
	//
	for(i=0;i<3;i++){
		m.dv[i]=0;
		m.ba[i]=0;
		m.bg[i]=0;
		m.fn[i]=0;
		m.rnin[i]=m.rnie[i] + m.rnen[i];
		m.cor[i] = 2 * m.rnie[i] + m.rnen[i];
	}
	m.pgps_time = igpsl.gps_time;
	m.sts = 1;
	m.gps_time = igpsl.gps_time;


	h.dt = IMU_SMPL_PERIOD;
	h.integ_timing_reference = 0;
	h.rmcnt_to_mspd = (unsigned int)round(m.dt/h.dt);
	h.rstcnt_to_mspd = h.rmcnt_to_mspd;
	h.acc_fn_cnt = 0;
	for(i=0;i<3;i++){
		h.acc_fn[i]=0;
		h.dang[i]=0;
		h.ba[i]=0;
		h.bg[i]=0;
	}


	h.gps_time = igpsl.gps_time;
	h.pgps_time = igpsl.gps_time;
	h.sts = 1;

	// reset no-zero_velocity-condition count to zero
	h.no_zvelcond_cnt = 0;
	h.max_mag_diff_accl = 0;
	h.min_mag_diff_accl = 0; 
	h.accl_mag_sum = 0;
	h.rot_rate_mag_sum = 0;
}

int		tframe_covar_tight(type_at_epoch *e)
{
	// position error del_lat, del_lon in rad
	// reset covar of dimension 17 for tightly-coupled GPS/SDINS
	// returns 1 if success
	// returns 0 if fail
	//---------------------

	int i,j, n;
	ColumnVector SS;
	double Sf, Sg, n1, n2;
	double h0, h2;
	double pscalefactor, qscalefactor;


	e->n = 15;
	//e->n = 16;
	e->numcr = 0;
	n = (e->n); 
	e->proc_cnt = 0;
	e->pre_hd = 0;

	pscalefactor = 1.0;
	qscalefactor = 1.0;

	for(j=0;j<=n;j++){
		for(i=0;i<=n;i++){
			e->P[j][i]=0.0;
		}
		e->Q[j]=0.0;
	}



	// SS
	SS.ReSize(n); SS = 0.0;
	SS(1) = pow(1000/e->rmh , 1.0);	// position error	//LimJH
	SS(2) = pow(1000/(e->rth)/(e->clat) , 1.0);						//LimJH
	SS(3) = pow(1000 , 1.0);						//LimJH
	SS(4) = pow(pscalefactor * 1 , 2.0);	// velocity error
	SS(5) = pow(pscalefactor * 1 , 2.0);	
	SS(6) = pow(pscalefactor * 1 , 2.0);	
	SS(7) = pow(pscalefactor *  2.0 * d2r , 2.0 );// roll
	SS(8) = SS(7);// pitch	
	SS(9) = pow( pscalefactor * (5.0 * d2r) , 2.0 ); //yaw(heading)	
	SS(10) = SS(11)  = SS(12)  = pow( pscalefactor * (0.05 * gm), 2.0);  //* accelerometer bias(horizontal)
	SS(13)= SS(14) = SS(15) = pow( pscalefactor * (0.5 * d2r ), 2.0);    //* gyro bias(horizontal) 
	//SS(16) = pow(pscalefactor * 0.1 , 2.0);	// velocity error	//LimJH
	//SS(17) = pow(pscalefactor * 1 , 2.0);						//LimJH
	//SS(18) = pow(pscalefactor * 1 , 2.0);						//LimJH
	//SS(13)=SS(14)=SS(15)=1; // Lever arm
	//SS(16)=1; // Time delay

	// Assign P, Q
	for(i=1 ; i<=n ; i++)	e->P[i-1][i-1] = SS(i);   // position, velocity, attitude


	//for(i=1 ; i<=2 ; i++)		e->Q[i-1] = 1*pow( qscalefactor * (5/e->rmh), 2.0); // Pos;			//LimJH
	e->Q[0] = pow(10/e->rmh, 2.0);							//lat
	e->Q[1] = pow(10/(e->rth)/(e->clat) , 2.0);				//lon
	e->Q[2] = pow(10, 2.0);									//hgt
	//for(i=4 ; i<=6 ; i++)		e->Q[i-1] = 1*pow( qscalefactor * (0.05*gm), 2.0); // Vel;
	for (i = 4; i <= 6; i++)	e->Q[i - 1] = 0.1;// 1 * pow(qscalefactor * (0.05*gm), 2.0); // Vel;
	for(i=7 ; i<=9 ; i++)		e->Q[i-1] = 1*pow( qscalefactor * (0.5 * d2r), 2.0); // Atti;	
	for(i=10 ; i<=12 ; i++)		e->Q[i-1] = 1*pow( ( qscalefactor * 0.01 * gm), 2.0); // rnAcc
	for(i=13 ; i<=15 ; i++)		e->Q[i-1] = 1*pow( ( qscalefactor * 0.1 * d2r ), 2.0); // rnGyro
	//e->Q[15] = (0.0001)*(0.0001)*1; // Vel;
	/*
	e->Q[0] = pow(1/e->rmh, 2.0);							//lat
	e->Q[1] = pow(0/(e->rth)/(e->clat) , 2.0);				//lon
	e->Q[2] = pow(0, 2.0);									//hgt
	//for(i=4 ; i<=6 ; i++)		e->Q[i-1] = 1*pow( qscalefactor * (0.05*gm), 2.0); // Vel;
	for(i=4 ; i<=6 ; i++)		e->Q[i-1] = 1*pow( qscalefactor * (0.02*gm), 2.0); // Vel;
	for(i=7 ; i<=9 ; i++)		e->Q[i-1] = 1*pow( qscalefactor * (0.2 * d2r), 2.0); // Atti;	
	for(i=10 ; i<=12 ; i++)		e->Q[i-1] = 1*pow( ( qscalefactor * 0.01 * gm), 2.0); // rnAcc
	for(i=13 ; i<=15 ; i++)		e->Q[i-1] = 1*pow( ( qscalefactor * 0.1 * d2r ), 2.0); // rnGyro
	*/
	

	// reset error states
	for(i=0 ; i<n ; i++){
		e->x[i] = 0;
		e->delx[i] = 0;
	}

	//////////////////////
	
	return 1;
}

//int		brm_avr_imu(type_hspd_sdins *h, type_raw_imu **raw, type_avr_imu *avr, int idx, int flag)
//{
//	// bias remove and average imu data
//	//
//	// returns 1 if success
//	// returns 0 if fail
//	int i, j;
//	double sum_fb[3];
//	double sum_rbib[3];
//
//	//flag = 1;
//
//	// shift & prepare imu data storage
//	// the larger the index i, the older the data is
//	for(i = ((int)(avr->msamp) - 1) ; i>0 ; i-- ){
//		for(j=0;j<3;j++){
//			avr->p_fb[i][j] = avr->p_fb[i-1][j];
//			avr->p_rbib[i][j] = avr->p_rbib[i-1][j];
//		}
//	}
//	// first index i = 0 indicates the data just arrived,
//	for(j=0;j<3;j++){
//		avr->p_fb[0][j] = raw[0]->fb[j] - h->ba[j];
//		avr->p_rbib[0][j] = raw[0]->rbib[j] - h->bg[j];
//	}
//	// compute average data using the stored data
//	for(j=0;j<3;j++){
//		sum_fb[j] = 0;
//		sum_rbib[j] = 0;
//	}
//	for(j=0;j<3;j++){
//		for( i=0 ; i < (avr->msamp) ; i++ ){
//			sum_fb[j] = sum_fb[j] + avr->p_fb[i][j];
//			sum_rbib[j] = sum_rbib[j] + avr->p_rbib[i][j];
//		}
//	}
//	for(j=0;j<3;j++){
//		if (flag)
//		{
//			avr->rbib[j] = sum_rbib[j] / (avr->msamp);
//			avr->fb[j] = raw[0]->fb[j] - h->ba[j];
//
//
//			//avr->rbib[j] = (sum_rbib[j] / (avr->msamp)) - h->bg[j];
//			//avr->fb[j] = (sum_fb[j] / avr->msamp) - h->ba[j];
//			
//			
//		}
//		else
//		{
//			avr->rbib[j] = raw[0]->rbib[j];
//			avr->fb[j] = raw[0]->fb[j];
//		}
//	}
//
//	avr->sts = 1; // allow external use
//	return 1;
//}

int		brm_avr_imu(type_hspd_sdins *h, type_raw_imu *raw, type_avr_imu *avr, int idx, int flag)
{
	// bias remove and average imu data
	//
	// returns 1 if success
	// returns 0 if fail
	int i, j;
	double sum_fb[3];
	double sum_rbib[3];

	//flag = 1;

	// shift & prepare imu data storage
	// the larger the index i, the older the data is
	for (i = ((int)(avr->msamp) - 1); i > 0; i--) {
		for (j = 0; j < 3; j++) {
			avr->p_fb[i][j] = avr->p_fb[i - 1][j];
			avr->p_rbib[i][j] = avr->p_rbib[i - 1][j];
		}
	}
	// first index i = 0 indicates the data just arrived,
	for (j = 0; j < 3; j++) {
		avr->p_fb[0][j] = raw->fb[j] - h->ba[j];
		avr->p_rbib[0][j] = raw->rbib[j] - h->bg[j];
	}
	// compute average data using the stored data
	for (j = 0; j < 3; j++) {
		sum_fb[j] = 0;
		sum_rbib[j] = 0;
	}
	for (j = 0; j < 3; j++) {
		for (i = 0; i < (avr->msamp); i++) {
			sum_fb[j] = sum_fb[j] + avr->p_fb[i][j];
			sum_rbib[j] = sum_rbib[j] + avr->p_rbib[i][j];
		}
	}
	for (j = 0; j < 3; j++) {
		if (flag)
		{
			avr->rbib[j] = sum_rbib[j] / (avr->msamp);
			avr->fb[j] = raw->fb[j] - h->ba[j];


			//avr->rbib[j] = (sum_rbib[j] / (avr->msamp)) - h->bg[j];
			//avr->fb[j] = (sum_fb[j] / avr->msamp) - h->ba[j];


		}
		else
		{
			avr->rbib[j] = raw->rbib[j];
			avr->fb[j] = raw->fb[j];
		}
	}

	avr->sts = 1; // allow external use
	return 1;
}

int		do_hspd_sdins_in_rx_seq(type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *e, type_avr_imu *s)//, int back_flag)
{
	// perform high speed sdins computation
	//
	// returns 1 if success
	// returns 0 if fail

	// h : pointer to high speed sdins data
	// m : pointer to medium speed sdins nav data
	// s : pointer to time-tagged imu data

	double rbin[3], dang[3], rv[3], tv[4], tq[4], r[10], adt;
	double fb[3], spcf_force_diff, rot_rate_diff;

	int delay_flag; // 1 if delayed measurements are to be used, 0 for nodelay 
	// usually use delay_flag=0

	delay_flag=0;//<====================================

	if(h->sts==0) perror("Error : Other Process is using structure hspd(detected in  do_hspd_sdins() )");
	h->sts = 0; // external use of this high speed sdins data is not allowed for a while

	// update hspd_sdins time tags using imu data
	h->pgps_time = h->gps_time; // save current time as previous time
	h->gps_time = s->gps_time; // read current time from imu data
	h->ce_time = s->ce_time;
	h->pcnt_frm_epoch = h->cnt_frm_epoch;
	h->cnt_frm_epoch = s->cnt_frm_epoch; 	
	h->idx_frm_epoch = h->idx_frm_epoch + 1; // increment index from epoch <=======

	// if first entry to this routine, disguise as if h is just previous information  
	if( h->gps_time == 0 ) h->gps_time = s->gps_time - h->dt;


	// NOW, ACTUALLY START THE HIGH SPEED SDINS COMPUTATION

	//***check gps_time using raw imu data, and compute adt
	adt = IMU_SMPL_PERIOD;//h->gps_time - h->pgps_time;
	//adt = h->gps_time - h->pgps_time;

	//if(h->integ_timing_reference == 0){
	//	adt = h->dt;
	//}

	s->sts = 0; // disable access to averaged IMU data

	// save sensor biases used using imu data
	s->ba[0]=h->ba[0];
	s->ba[1]=h->ba[1];
	s->ba[2]=h->ba[2];
	s->bg[0]=h->bg[0];
	s->bg[1]=h->bg[1];
	s->bg[2]=h->bg[2];

	// check zero velocity condition 
	// use accel meas.
	spcf_force_diff = sqrt(s->fb[0]*s->fb[0]+s->fb[1]*s->fb[1]+s->fb[2]*s->fb[2]) - m->g;
	if(spcf_force_diff<0) spcf_force_diff = -spcf_force_diff;
	h->accl_mag_sum += spcf_force_diff;
	// use gyto meas.
	rot_rate_diff = sqrt(s->rbib[0]*s->rbib[0]+s->rbib[1]*s->rbib[1]+s->rbib[2]*s->rbib[2]);// - m->er;
	if(rot_rate_diff<0) rot_rate_diff = -rot_rate_diff;
	h->rot_rate_mag_sum += rot_rate_diff;
	// other accel related info
	if((spcf_force_diff>ZV_SF_OFS)||(spcf_force_diff< -ZV_SF_OFS)) h->no_zvelcond_cnt++; 
	if(spcf_force_diff > h->max_mag_diff_accl) h->max_mag_diff_accl = spcf_force_diff;
	if(spcf_force_diff < h->min_mag_diff_accl) h->min_mag_diff_accl = spcf_force_diff;
	// END of check zero velocity condition 

	// rbin=c'*rnin
	rbin[0] = h->c[0] * m->rnin[0] + h->c[3] * m->rnin[1] + h->c[6] * m->rnin[2];
	rbin[1] = h->c[1] * m->rnin[0] + h->c[4] * m->rnin[1] + h->c[7] * m->rnin[2];
	rbin[2] = h->c[2] * m->rnin[0] + h->c[5] * m->rnin[1] + h->c[8] * m->rnin[2];

	// compute nav-body incremental angle
	dang[0] = (s->rbib[0]  - rbin[0]) * adt;
	dang[1] = (s->rbib[1]  - rbin[1]) * adt;
	dang[2] = (s->rbib[2]  - rbin[2]) * adt;

	// compute rotation vector using previous data 
	rv[0] = dang[0] - (h->dang[2] * dang[1] - h->dang[1] * dang[2]) /12.;
	rv[1] = dang[1] + (h->dang[2] * dang[0] - h->dang[0] * dang[2]) /12.;
	rv[2] = dang[2] - (h->dang[1] * dang[0] - h->dang[0] * dang[1]) /12.;

	// save current incremental angle for next computation
	h->dang[0]=dang[0];
	h->dang[1]=dang[1];
	h->dang[2]=dang[2];

	// compute delta quaternion 
	tv[0] = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
	tv[1] = 0.5 - (0.0208333333 - tv[0] / 3840) * tv[0];
	tv[2] = 1 - (0.125 - tv[0] /384) * tv[0];

	// quaternion update 
	tq[0] = tv[2] * h->q[0] - tv[1] * (rv[0] * h->q[1] + rv[1] * h->q[2] + rv[2] * h->q[3]);
	tq[1] = tv[2] * h->q[1] + tv[1] * (rv[0] * h->q[0] + rv[2] * h->q[2] - rv[1] * h->q[3]);
	tq[2] = tv[2] * h->q[2] + tv[1] * (rv[1] * h->q[0] - rv[2] * h->q[1] + rv[0] * h->q[3]);
	tq[3] = tv[2] * h->q[3] + tv[1] * (rv[2] * h->q[0] + rv[1] * h->q[1] - rv[0] * h->q[2]);

	// quaternion normalization 
	tv[3] = 1.5 - 0.5*(tq[0] * tq[0] + tq[1] * tq[1] + tq[2] * tq[2] + tq[3] * tq[3]);
	h->q[0] = tv[3]*tq[0];
	h->q[1] = tv[3]*tq[1];
	h->q[2] = tv[3]*tq[2];
	h->q[3] = tv[3]*tq[3];

	// compute body to navigation transformation matrix 
	r[0] = h->q[0] * h->q[0];
	r[1] = h->q[1] * h->q[1];
	r[2] = h->q[2] * h->q[2];
	r[3] = h->q[3] * h->q[3];
	r[4] = h->q[0] * h->q[1];
	r[5] = h->q[0] * h->q[2];
	r[6] = h->q[0] * h->q[3];
	r[7] = h->q[1] * h->q[2];
	r[8] = h->q[1] * h->q[3];
	r[9] = h->q[2] * h->q[3];

	h->c[0] = r[0] + r[1] - r[2] - r[3];
	h->c[1] = 2 * (r[7] - r[6]);
	h->c[2] = 2 * (r[5] + r[8]);
	h->c[3] = 2 * (r[6] + r[7]);
	h->c[4] = r[0] - r[1] + r[2] - r[3];
	h->c[5] = 2 * (r[9] - r[4]);
	h->c[6] = 2 * (r[8] - r[5]);
	h->c[7] = 2 * (r[9] + r[4]);
	h->c[8] = r[0] - r[1] - r[2] + r[3];

	h->headd =atan2( h->c[3] , h->c[0] )*r2d;


	h->dang[0] = dang[0];
	h->dang[1] = dang[1];
	h->dang[2] = dang[2];

	/* accumulate acceleration along NED frame */
	fb[0] = s->fb[0];
	fb[1] = s->fb[1];
	fb[2] = s->fb[2];

	// OPTION 1 : sculling algorithm

	h->acc_fn[0] = h->acc_fn[0] + h->c[0] * fb[0] + h->c[1] * fb[1] + h->c[2] * fb[2];
	h->acc_fn[1] = h->acc_fn[1] + h->c[3] * fb[0] + h->c[4] * fb[1] + h->c[5] * fb[2];
	h->acc_fn[2] = h->acc_fn[2] + h->c[6] * fb[0] + h->c[7] * fb[1] + h->c[8] * fb[2];
	h->acc_fn_cnt = h->acc_fn_cnt + 1;

	h->rmcnt_to_mspd = h->rmcnt_to_mspd - 1; // decrement count to next do_mspd_sdins() execution

	////////////////////////////////////////////////////////////////////////////////////////////////////
	e->lmdist[0];
	// if PPS did not occur but the remaining count to mspd is zero, automatically start mspd
	if(h->rmcnt_to_mspd==0){
		if( do_mspd_sdins(h,m,e) == 0){ // if error occurres performing do_mspd_sdins()
			perror("Error : reported in performing do_mspd_sdins() by do_hspd_sdins()");
			return 0;
		}
	}

	h->sts = 1; // external use of this high speed sdins data is allowed





	return 0;
}

int		do_mspd_sdins(type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *pps)
{
	// perform medium speed sdins computation
	//
	// returns 1 if success
	// returns 0 if fail

	// note : this routine is automatically called by do_mspd_sdins routine.

	// h : pointer to high speed sdins data
	// m : pointer to medium speed sdins nav data
	// s : pointer to time-tagged imu data

	double dlat, dlon, dhgt, dv[3], vdt, tmpdt, oes, g0;
	int cnt, j;


	if(m->sts==0) perror("Error : Other Process is using structure mspd(detected in  do_mspd_sdins() )");
	m->sts = 0; // external use of this medium speed sdins data is not allowed for a while

	// update mspd_sdins time tags using imu data
	m->pgps_time = m->gps_time; // save current time as previous time
	m->gps_time = h->gps_time; // read current time from imu data
	m->ce_time = h->ce_time;
	m->cnt_frm_epoch = h->cnt_frm_epoch; 



	// if first entry to this routine, disguise as if h is just previous information  
	if( m->gps_time == 0 ) m->gps_time = h->gps_time - m->dt;

	// save sensor biases used using high speed sdins data
	for(cnt=0; cnt<3; cnt++){
		m->ba[cnt] = h->ba[cnt];
		m->bg[cnt] = h->bg[cnt];
	}
	// save attitude information using high speed sdins data
	for(cnt=0; cnt<9; cnt++){ m->c[cnt] = h->c[cnt];}
	for(cnt=0; cnt<4; cnt++){ m->q[cnt] = h->q[cnt];}

	/* compute merdian & tangential earth radius */
	m->slat = sin(m->lat);
	m->clat = cos(m->lat);
	m->slat2 = (m->slat)*(m->slat);

	// updated code (m)
	oes = sqrt(1-epe*epe*m->slat2);
	m->rm = epam*(1-epe*epe)/(oes*oes*oes);
	m->rt = epam/oes;
	g0 = gm*(1+0.0053024*m->slat2 - 0.0000058*sin(2*m->lat)*sin(2*m->lat)); // latitude correction of local gravity 
	m->g = g0 - (3.0877e-6 - 0.0044e-6*m->slat2)*m->hgt + 0.072e-12*(m->hgt*m->hgt); // height correction of local gravity 

	/* compute latitude rate longitude rate and vertical velocity */
	m->rmh = m->rm + m->hgt;
	m->rth = m->rt + m->hgt;

	/* compute latitude rate longitude rate and vertical velocity */
	dlat = m->v[0] / (m->rmh);
	dlon = m->v[1] / ((m->rth) * (m->clat));
	dhgt = - m->v[2];

	/* compute earth rate along n-frame */
	m->rnie[0] = m->er * m->clat;
	m->rnie[1] = 0;	
	m->rnie[2] = - m->er * m->slat;

	/* compute transportation rate */
	m->rnen[0] = m->dlon * m->clat;
	m->rnen[1] = - m->dlat;
	m->rnen[2] = - m->dlon * m->slat;

	/* compute n-frame torquing rate */
	m->rnin[0] = m->rnie[0] + m->rnen[0];
	m->rnin[1] = m->rnie[1] + m->rnen[1];
	m->rnin[2] = m->rnie[2] + m->rnen[2];

	/* compute coriolis term */
	m->cor[0] = 2 * m->rnie[0] + m->rnen[0];
	m->cor[1] = 2 * m->rnie[1] + m->rnen[1];
	m->cor[2] = 2 * m->rnie[2] + m->rnen[2];	

	/* compute averaged specific force along NED frame */
	// OPTION 1: sculling algorithm 

	if(h->acc_fn_cnt != 0){
		m->fn[0] = h->acc_fn[0] / h->acc_fn_cnt;
		m->fn[1] = h->acc_fn[1] / h->acc_fn_cnt;
		m->fn[2] = h->acc_fn[2] / h->acc_fn_cnt;
	}
	else{
		perror("specific force accumulation count at medium speed is zero");
	}


	// compute dv/dt w.r.t. nav frame
	dv[0] = m->fn[0]			+ m->v[1] * m->cor[2] - m->v[2] * m->cor[1];
	dv[1] = m->fn[1]			- m->v[0] * m->cor[2] + m->v[2] * m->cor[0];
	dv[2] = m->fn[2] + m->g		+ m->v[0] * m->cor[1] - m->v[1] * m->cor[0]; // for high-quality accel. measuring specific force

	//vdt = ((double) h->acc_fn_cnt)*h->dt; 
	vdt = 0.01;

	if(vdt>1){
		vdt = vdt -1;
		perror("Warning : vdt is too large in do_mspd_sdins()");
	}
	else if(vdt < 0){
		vdt = vdt + SECONDS_IN_WEEK;
		perror("Warning : vdt is negative due to week cross-over in do_mspd_sdins()");
	}


	/* compute velocity along NED frame */
	m->v[0] = m->v[0] + 0.5 * (dv[0] + m->dv[0]) * vdt;
	m->v[1] = m->v[1] + 0.5 * (dv[1] + m->dv[1]) * vdt;
	m->v[2] = m->v[2] + 0.5 * (dv[2] + m->dv[2]) * vdt;

	m->dlat = m->v[0] / (m->rmh);
	m->dlon = m->v[1] / ((m->rth) * (m->clat));
	m->dhgt = - m->v[2];

	m->lat = m->lat + 0.5*(dlat + m->dlat) * vdt;
	m->lon = m->lon + 0.5*(dlon + m->dlon) * vdt;
	m->hgt = m->hgt + 0.5*(dhgt + m->dhgt) * vdt;

	m->dv[0] = dv[0];
	m->dv[1] = dv[1];
	m->dv[2] = dv[2];

	// reset remaining count to next medium speed sdins computation
	h->rmcnt_to_mspd = h->rstcnt_to_mspd;

	// reset accumulated specific force variables of hspd 
	h->acc_fn[0] = 0;
	h->acc_fn[1] = 0;
	h->acc_fn[2] = 0;
	h->acc_fn_cnt = 0;

	m->sts = 1; // external use of this medium speed sdins data is allowed


	/*---------------------------------------*/
	/*---------------------------------------*/
	/*---------------------------------------*/

	//printf("Medium speed check : ");
	//printf("lat = %f  ",m->lat*r2d);
	//printf("lon = %f  ",m->lon*r2d);
	//printf("hgt = %f\n",m->hgt);

	/*---------------------------------------*/
	/*---------------------------------------*/
	/*---------------------------------------*/

	return 1;
}

int		tframe_do_at_gps_tight_rx_in_rx_seq(type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *e, type_avr_imu *s, type_gps_sol *tgps, int fava_gps)
{
	// returns 1 if success
	// returns 0 if fail

	double mdt;
	Matrix uxyzb, temp;

	if(h->acc_fn_cnt!=0) do_mspd_sdins(h, m, e);

	save_shm_for_filter(s, h, m, e);
	
	e->proc_cnt ++;

	e->m_up_type = 8;
	//e->m_up_type = 81; 

	mdt = m->dt;
	tgps->pre_HDOP = tgps->HDOP;
		

	if(tframe_do_propa_tight_newmat(e, m)==0){ //<===
		perror("ERROR : in propagation of error covar is reported in subroutine do_at_PPS_sdins_file_nodelay()");
		return 0;
	}


	if(e->m_up_type != -1){ 
		if(tframe_do_mupdate_tight_newmat(h,e, tgps)==0){ //<===
			perror("ERROR : in meas. update is reported in subroutine do_at_PPS_sdins_file_nodelay()]");
			return 0;
		}
	}


	if(e->p_sts == 1){ // in case when the previous gps data was used for meas. update,

		tframe_compen_hspd_sdins(h,e, 1); // compensate e->delx to h,
		tframe_compen_mspd_sdins(m,e, 1); // compensate e->delx to 

		e->p_sts = 0;
		e->delay  = e->delay;//  - e->delx[15];
	}

	e->pre_hd = tgps->hd;
	tgps->pre_hgt = tgps->hgt;

	// reset no-zero_velocity-condition count to zero
	h->no_zvelcond_cnt = 0;
	h->max_mag_diff_accl = 0;
	h->min_mag_diff_accl = 0; 
	h->accl_mag_sum = 0;
	h->rot_rate_mag_sum = 0;

	// prevent error compensation at PPS
	e->p_sts = 0;
	
	// in fact, error compensation with delayed meas. update is already done in this routine
	return 1;
}

void	save_shm_for_filter(type_avr_imu *s, type_hspd_sdins *h, type_mspd_sdins *m, type_at_epoch *e){
	// save hspd and mspd for filter propa & update

	int i;
	e->sts = 0; // mark that pointer e is used by main process 
	e->gps_time = m->gps_time;
	e->pgps_time = m->pgps_time;
	e->ce_time = m->ce_time;
	e->cnt_frm_epoch = m->cnt_frm_epoch;
	e->dt = m->dt;
	for(i=0;i<4;i++) e->q[i] = m->q[i];
	for(i=0;i<9;i++) e->c[i] = m->c[i];
	for(i=0;i<3;i++){
		e->fn[i] = m->fn[i];
		e->v[i] = m->v[i];
		e->dv[i] = m->dv[i];
		e->ba[i] = h->ba[i];
		e->bg[i] = h->bg[i];
		e->rnin[i] = m->rnin[i];
		e->rnie[i] = m->rnie[i];
		e->rnen[i] = m->rnen[i];
		e->cor[i] = m->cor[i];
	}
	e->lat = m->lat;
	e->lon = m->lon; 
	e->lat0 = m->lat0;
	e->lon0 = m->lon0; 
	e->smlat = m->smlat;
	e->smlon = m->smlon; 
	e->hgt = m->hgt;

	e->dlat = m->dlat;
	e->dlon = m->dlon;
	e->dhgt = m->dhgt;
	e->slat = m->slat;
	e->clat = m->clat;
	e->slat2 = m->slat2;
	e->g = m->g;
	e->er = m->er;
	e->rm = m->rm;
	e->rt = m->rt;
	e->rmh = m->rmh;
	e->rth = m->rth;
	e->gps_time_last_propa = m->gps_time;

	// copy high speed sdins information
	for(i=0; i<3; i++) e->rbib[i] = s->rbib[i];
	e->accl_mag_sum = h->accl_mag_sum;
	e->rot_rate_mag_sum = h->rot_rate_mag_sum;
	e->no_zvelcond_cnt = h->no_zvelcond_cnt;
	// set e free to use
	e->sts = 1;
}

int		tframe_do_propa_tight_newmat(type_at_epoch *e, type_mspd_sdins *m)
{
	// propagate 17 state GPS TC-coupled Kalman filter
	//
	// returns 1 if success
	// returns 0 if fail

	int i,j, k; // row, column, and mtx. mult. index
	int n; // (order of the Kalman filter) - 1
	double ff[AMAX][AMAX]; // pointer to system coefficient matrix
	double p1[AMAX][AMAX]; // pointer to temp (n+1)x(n+1) matrix 
	double cdelx[AMAX];
	double kadt;

	double C11, C12, C13, C21, C22, C23, C31, C32, C33;
	double S11, S12, S13, S21, S22, S23, S31, S32, S33;
	double GG1, GG2, GG3, GG4;
	//double rnie[3], rnen[3], rnin[3], fn[3], v[3], Slat, Clat, Slon, Clon, Tlat, hgt, Rmh, Rth, RRM, RRT;
	double rnie[3], rnen[3], rnin[3], cor[3], fn[3], v[3], Slat, Clat, Slon, Clon, Tlat, hgt, Rmh, Rth, dRmL, dRtL;
	double epe2;

	Matrix P, Pb, Q, F, Phi, H, R, S, RC, SC;
	Matrix x, xb; // accumulated error for feedforward correction


	//$ check & mark the status of data structure e
	if(e->sts == 0){
		perror("Error : type_at_epoch is used by two different main process/detected in do_propa()");
		return 0;
	}
	e->sts = 0; // mark that main algorithm is using e

	//  e : pointer to type_at_epoch
	//  e->msd : type_mspd_sdins

	// (Kalman filter order)
	n = e->n; // KF dimension is fixed for tightly-coupled configuration
	//{ option 1 for filter integration time interval
	kadt = 0.01; // propagation is occurred at every epoch
	// option 2 for filter integration time interval
	//kadt = fmod(e->gps_time,SECONDS_IN_WEEK) - fmod(e->gps_time_last_propa,SECONDS_IN_WEEK);


	//kadt = 1;
	e->gps_time_last_propa = e->gps_time;

	//} end kadt

	for(i=0;i<n;i++){
		//cdelx[i]=0;
		for(j=0;j<n;j++){
			ff[i][j]=0;
			p1[i][j]=0;
			e->phi[i][j]=0;
		}
	}

	//$$ system matrix construction

	//assign variables
	for(i=0;i<3;i++){
		rnie[i]=m->rnie[i];
		rnen[i]=m->rnen[i];
		rnin[i]=m->rnin[i];
		cor[i] = m->cor[i];
		fn[i]=m->fn[i];
		v[i]=m->v[i] - e->x[i+3];
	}

	// assign variables
	Slat=sin(m->lat - e->x[0]);  
	Clat=cos(m->lat - e->x[0]);
	Slon=sin(m->lon - e->x[1]);
	Clon=cos(m->lon - e->x[1]);
	Tlat=tan(m->lat - e->x[0]);
	hgt=m->hgt - e->x[2];
	epe2 = epe*epe;
	Rmh=m->rmh - e->x[2];
	Rth=m->rth - e->x[2];

	//RRM = 3*RE*EC*(1-EC)*Slat*Clat * pow( (1-EC*Slat*Slat) , -2.5 );
	//RRT = RE*EC*Slat*Clat * pow( (1-EC*Slat*Slat) , -1.5 );
	dRmL = 3*epam* epe2 *(1- epe2)*Slat*Clat * pow( (1-epe2*Slat*Slat) , -2.5 );
	dRtL = epam* epe2 *Slat*Clat * pow( (1-epe2*Slat*Slat) , -1.5 );

	// tranformation matrix
	C11 = m->c[0];
	C12 = m->c[1];
	C13 = m->c[2];
	C21 = m->c[3];
	C22 = m->c[4];
	C23 = m->c[5];
	C31 = m->c[6];
	C32 = m->c[7];
	C33 = m->c[8];
	S11 = -m->c[0];
	S12 = -m->c[1];
	S13 = -m->c[2];
	S21 = -m->c[3];
	S22 = -m->c[4];
	S23 = -m->c[5];
	S31 = -m->c[6];
	S32 = -m->c[7];
	S33 = -m->c[8];

	// pos<-pos
	ff[0][0] = rnen[1] * dRmL / Rmh;
	ff[0][2] = rnen[1] / Rmh;
	ff[1][0] = rnen[0] / Clat * (Tlat - dRtL/Rth);
	ff[1][2] = -rnen[0] / Clat / (Rth);

	// pos<-vel
	ff[0][3] = 1.0 / Rmh;
	ff[1][4] = 1.0 / Rth / Clat;
	ff[2][5] = -1.0;


	//vel<-pos      
	GG1 = 0.0518590869785989; 
	GG2 = -3.0877e-6;
	GG3 = 4.3e-9;
	GG4 = 2.194559929e-13;

	ff[3][0] = -v[1]*(2*rnie[0]+rnen[0]/Clat/Clat) + v[2]*rnen[1]*dRmL/Rmh - rnen[0]*rnen[2]*dRtL;
	ff[3][2] = rnen[1]*v[2]/Rmh - rnen[0]*rnen[2];
	ff[4][0] = -v[2]*(-2*rnie[2]+rnen[0]*dRtL/Rth) + v[0]*(2*rnie[0]+rnen[0]/Clat/Clat+rnen[2]*dRtL/Rth);
	ff[4][2] = -rnen[0]*v[2]/Rth + rnen[2]*v[0]/Rth;  
	ff[5][0] = -2*rnie[2]*v[1] + rnen[0]*rnen[0]*dRtL + rnen[1]*rnen[1]*dRmL + 2*Slat*Clat*(GG1+GG3*hgt);
	ff[5][2] = rnen[0] * rnen[0] + rnen[1] * rnen[1] +GG2 + GG3*Slat*Slat + 2 * GG4*hgt;
	//m->g = GE + GG1*(m->slat)*(m->slat) + GG2*(m->lon) + GG3*(m->lon)*(m->slat)*(m->slat) + GG4*(m->lon)*(m->lon);


	//vel<-vel
	ff[3][3] = v[2] / Rmh;
	ff[3][4] = 2*(rnie[2] + rnen[2]);
	ff[3][5] = -rnen[1];
	ff[4][3] = -2*rnie[2] - rnen[2];
	ff[4][4] = v[2]/Rth + v[0]*Tlat/Rth;
	ff[4][5] = 2*rnie[0] + rnen[0];  
	ff[5][3] = 2*rnen[1];
	ff[5][4] = -2*(rnie[0]+rnen[0]);  

	//vel<-atti
	ff[3][7] = -fn[2];
	ff[3][8] = fn[1];
	ff[4][6] = fn[2];
	ff[4][8] = -fn[0];
	ff[5][6] = -fn[1];
	ff[5][7] = fn[0];


	//atti<-pos
	ff[6][0] = rnie[2] - rnen[0]*dRtL / Rth;
	ff[6][2] = -rnen[0] / Rth;
	ff[7][0] = -(rnen[1] * dRmL / Rmh);
	ff[7][2] = -(rnen[1] / Rmh);
	ff[8][0] = -rnie[0] - rnen[0] / (Clat*Clat) - rnen[2] * dRtL / Rth;
	ff[8][2] = -rnen[2] / Rth;    


	//atti<-vel      
	ff[6][4] = 1.0 / Rth;
	ff[7][3] = -1.0 / Rmh;
	ff[8][4] = -Tlat / Rth;


	//atti<-atti
	ff[6][7] = rnie[2] + rnen[2];
	ff[6][8] = -rnen[1];
	ff[7][6] = -rnie[2] - rnen[2];
	ff[7][8] = rnie[0] + rnen[0];
	ff[8][6] = rnen[1];
	ff[8][7] = -rnie[0] - rnen[0];

	// f24 : (3,9) // vel <- accel bias
	ff[3][9] =  C11; ff[3][10] =  C12; ff[3][11] =  C13;
	ff[4][9] =  C21; ff[4][10] =  C22; ff[4][11] =  C23;
	ff[5][9] =  C31; ff[5][10] =  C32; ff[5][11] =  C33;

	// atti <- gyro bias
	ff[6][12] = S11;
	ff[6][13] = S12;
	ff[6][14] = S13;
	ff[7][12] = S21;
	ff[7][13] = S22;
	ff[7][14] = S23;
	ff[8][12] = S31;
	ff[8][13] = S32;
	ff[8][14] = S33;

	// for injected inertial senfor markov error 
	//for(i=9;i<15;i++) ff[i][i] = -BETASEN;

	// clock bias <- rate
	//ff[15][16] = 1;

	//$$ error covariance propagation

	// F
	F.ReSize(n,n); F=0.0;
	//Srint(F);
	for(j = 0;j < n;j++){
		for(k = 0;k < n;k++){
			F(j+1,k+1) = ff[j][k];
		}
	}


	// P, Q, Phi, x
	P.ReSize(n,n); P = 0.0;
	Q.ReSize(n,n); Q = 0.0;
	Phi.ReSize(n,n); Phi = 0.0;
	x.ReSize(n,1); x = 0.0;
	
	for(j = 0;j < n;j++){
		x(j+1,1) = e->x[j];//<===
		for(k = 0;k < n;k++){
			P(j+1,k+1) = e->P[j][k];
			if(j == k){
				e->phi[j][k] = 1.0 + ff[j][k]*kadt;
				Phi(j+1,k+1) = 1.0 + ff[j][k]*kadt;
				Q(j+1,k+1) = e->Q[j];
			}
			else{
				e->phi[j][k] = ff[j][k]*kadt;
				Phi(j+1,k+1) = ff[j][k]*kadt;
			}
		}
	}

	// xb <===
	xb = Phi*x;
	for(j = 0;j < n;j++){
		e->x[j] = xb(j+1,1);
	}

	// Pb
	Pb = Phi*P*Phi.t() + Q*kadt;
	for(j = 0;j < n;j++){
		for(k = 0;k < n;k++){
			e->P[j][k] = Pb(j+1,k+1);
		}
	}

	// delx = Phi*delx // 
	for(i = 0;i < n;i++){
		for(j = 0;j < n;j++){
			cdelx[i] += e->phi[i][j]*e->delx[j];
		}
	}

	e->sts = 1; // mark that filter propagation is done

	return 1; // report subroutine success
}

int		tframe_do_mupdate_tight_newmat(type_hspd_sdins *h, type_at_epoch *e, type_gps_sol *tgps)
{

	// returns 1 if success
	// returns 0 if fail

	int i, j, k,n, m_up_type;// m = (dimension of measurement vector) - 1;
	Matrix t33(3,3), t31(3,1), t3n; // temporary matrix
	Matrix z, R; // measurement & measurement std
	Matrix S, H;
	Matrix P, Pup;
	Matrix dx;
	Matrix K, IKH;
	Matrix Ce2n(3,3), Cn2e(3,3), Rxl(3,3), ullh_m(3,1), uxyz_em(3,1), uvel_nf(3,1), uvel_em(3,1), svxyz_em(3,1), svvel_em(3,1),Cb2n(3,3),Giro(3,1);
	Matrix xyzdiff_em(3,1), LOSe(3,1), LOSt_e(1,3), LOSn(3,1), LOSt_n(1,3), veldiff_ef(3,1),Dist(3,1),CrGD(3,1);;
	//double Slat, Clat, Slon, Clon, Tlat, hgt, epe2, Rmh, Rth;
	double  Slat, Clat, Slon, Clon, Tlat, hgt, epe2, Rm, Rt, Rmh, Rth, dRmL, dRtL, tmp;
	double	sig_pr, sig_dppl, sig_zvel,giro[3];


	m_up_type = e->m_up_type;


	//$ (Kalman filter order)
	//n = (int) (e->n); // system dimension
	n=15;
	for(i = 0;i < n;i++) e->delx[i] = 0; // make del-x as zero vector

	e->sts = 0; // mark that main algorithm is using e

	e->lmdist[0] = 0.0;
	e->lmdist[1] = 0.0;
	e->lmdist[2] = 0.0;

	giro[0] = e->rbib[1]*e->lmdist[2] - e->rbib[2]*e->lmdist[1];
	giro[1] = e->rbib[2]*e->lmdist[0] - e->rbib[0]*e->lmdist[2];
	giro[2] = e->rbib[0]*e->lmdist[1] - e->rbib[1]*e->lmdist[0]; 

	CrGD << giro[0] 
	<<	giro[1]
	<< giro[2];

	Dist << e->lmdist[0] << e->lmdist[1] << e->lmdist[2];
	Giro << e->rbib[0] <<  e->rbib[1] <<  e->rbib[2];

	// transformation matrices Cb2n
	Cb2n << e->c[0]   <<  e->c[1]   << e->c[2]         
	<<  e->c[3]  <<  e->c[4]   << e->c[5]               
	<<  e->c[6]  <<  e->c[7]   << e->c[8];

	/////////////////////////////////////////////// USER INFO START
	// assign variables
	Slat=sin(e->lat); 
	Clat=cos(e->lat);
	Slon=sin(e->lon);
	Clon=cos(e->lon);
	Tlat=tan(e->lat);
	hgt= e->hgt;
	epe2 = epe*epe;
	Rm = e->rm;
	Rt = e->rt;
	Rmh = e->rmh;
	Rth = e->rth;
	
	dRmL = 3*epam* epe2 *(1- epe2)*Slat*Clat * pow( (1-epe2*Slat*Slat) , -2.5 );
	dRtL = epam* epe2 *Slat*Clat * pow( (1-epe2*Slat*Slat) , -1.5 );

	// transformation matrices Ce2n, Cn2e
	Ce2n << -Slat*Clon  <<  -Slat*Slon   << Clat         
		<< -Slon         <<  Clon           << 0               
		<< -Clat*Clon  <<  -Clat*Slon   << -Slat; 
	Cn2e = Ce2n.t(); 	

	// transformation matrix	Rxl
	Rxl(1,1) = dRtL*Clat*Clon - Rth * Slat * Clon; 
	Rxl(1,2) = -Rth*Clat*Slon;
	Rxl(1,3) = Clat * Clon;  
	Rxl(2,1) = dRtL*Clat*Slon - Rth * Slat * Slon;
	Rxl(2,2) = Rth * Clat * Clon;
	Rxl(2,3) = Clat * Slon ;
	Rxl(3,1) = dRtL * (1-epe2) * Slat + (dRtL*(1-epe2) + hgt)*Clat;
	Rxl(3,2) = 0;
	Rxl(3,3) = Slat;  

	//// xyze in m
	//ullh_m.ReSize(3,1); ullh_m << (e->lat-e->x[0]) << (e->lon-e->x[1]) << (e->hgt-e->x[2])*f2m;
	//uxyz_em  =  llh2xyz(ullh_m);
	//// vn in ft
	//uvel_nf.ReSize(3,1); uvel_nf << (e->v[0]-e->x[3]) << (e->v[1]-e->x[4]) << (e->v[2]-e->x[5]);
	//// ve in m
	//uvel_em = f2m*Cn2e*uvel_nf;
	/////////////////////////////////////////////// USER INFO FINISH


	//sig_pr = 60;// meters
	//sig_dppl = 0.1;// m/sec
	//sig_zvel = 0.01;// m/sec 

	//TRACE1("m_up_type = %d\n",m_up_type);

	Matrix uvelr, vn(3,1);
	uvelr.ReSize(3,1);

	uvelr(1,1) = tgps->vel[0];
	uvelr(2,1) = tgps->vel[1];
	uvelr(3,1) = tgps->vel[2];

	//vn = Ce2n*uvelr;
	vn = uvelr;
	/*vn(1,1) = tgps->vel_horz*cos(tgps->hd*d2r);
	vn(2,1) = tgps->vel_horz*sin(tgps->hd*d2r);
	vn(3,1) = 0;*/


//PrintT(vn);



	//double acc_r = 0.1;
	double acc_r = 0.01;
	double obd_r = 0.04;
	
	double yaw = atan2(h->c[3] , h->c[0]);
	if (yaw < 0)
		yaw += 360;

	//m_up_type = 9;
	if(m_up_type == 8){// 8 for gps POS, VEL, and IMU Atti update only : rotation by GYRO

		//// z
		z.ReSize(6,1);

		z(1,1) = (e->lat - (tgps->lat)*d2r);
		z(2,1) = (e->lon - (tgps->lon)*d2r);
		z(3,1) = e->hgt - (tgps->hgt); 
		z(4,1) = e->v[0] - vn(1,1);
		z(5,1) = e->v[1] - vn(2,1);
		z(6,1) = e->v[2] - vn(3,1);// note the sign "-(-) = +"

		//PrintT(z);
		// H
		H.ReSize(6,n); H=0.0;
		for(j=0;j<6;j++) H(j+1,j+1)=1.0;

		R.ReSize(6,6); R=0.0;
		R(1,1) = pow(1/e->rmh, 2.0);				//lat
		R(2,2) = pow(1/(e->rth)/(e->clat) , 2.0);				//lon
		R(3,3) = pow(2, 2.0);				//hgt
		for(j=3;j<6;j++) R(j+1,j+1)= acc_r*acc_r;
			
		// S
		S.ReSize(6,n); S=0.0;
		printf("+++++++++++++++++++++++++++++++++++++ 8 for gps POS, VEL update only\n");

	}
	
	else {
		perror("m_up_type is not properly set up in do_mupdate()");
		e->p_sts = 0; // mark that no measurement update occurred
		e->sts = 1; // make e accessable by external  
		return 0;
	}

	//$ Kalman gain & error covar. mtx. update

	//FILE *fp_p = fopen("matrix_p.txt","a");

	P.ReSize(n,n); 
	for(j=0;j<n;j++){
		//if (j<3)
		//	fprintf(fp_p,"%f\t%d\t",e->gps_time,m_up_type);
		for(k=0;k<n;k++){
			P(j+1,k+1) = e->P[j][k];
			//if (k<3 && j<3)
			//	fprintf(fp_p,"%e\t",(P(j+1,k+1)));
		}
		//fprintf(fp_p,"\n");
	}
	//fclose(fp_p);
	t33 = H*P*H.t() +R;
	t33 = t33.i();
	K = P*H.t()*t33;
	IKH = -K*H; 
	for(j=0;j<n;j++) IKH(j+1,j+1) = IKH(j+1,j+1) + 1;
	Pup = IKH*P*IKH.t() + K*R*K.t();

	//FILE *fp_pup = fopen("matrix_pup.txt","a");
	for(j=0;j<n;j++){
		//fprintf(fp_pup,"%f\t%d\t",e->gps_time,m_up_type);
		for(k=0;k<n;k++){
			e->P[j][k] = Pup(j+1,k+1);
			//fprintf(fp_pup,"%e\t",(Pup(j+1,k+1)));
		}
		//fprintf(fp_pup,"\n");
	}
		//fclose(fp_pup);
	// compute measurement-updated del-x & err. covar. mtx. with no delay  
	dx = K*z;
	for(j = 0;j < n;j++){
		e->delx[j] = dx(j+1,1);
	}
	tgps->delay = tgps->delay;// + e->delx[15];
	//PrintT(dx);

	e->gps_time_last_meas_data = fmod(tgps->gps_time,SECONDS_IN_WEEK);
	e->p_sts = 1; // mark that measurement update is done
	e->sts = 1; // make e accessable by external

	return 1; // report success
}

int		tframe_compen_hspd_sdins(type_hspd_sdins *h, type_at_epoch *e, int flag)
{
	// compensate e->delx to h
	// compensate high speed nav varialbes 
	// returns 1 if success
	// returns 0 if fail

	int sts_save, i;
	double kq[4];
	double kpsn, kpse, kpsd;

	sts_save = h->sts;
	h->sts = -1;
		
	kpsn = e->delx[6];						/* attitude */
	kpse = e->delx[7];
	kpsd = e->delx[8];

	double tmpc, RPY[3], relpos[3];
	tmpc = 1 - pow(h->c[6], 2.0);
	tmpc = sqrt(tmpc);
	RPY[0] = atan2(h->c[7], h->c[8]);
	RPY[1] = atan2(h->c[6], tmpc);
	RPY[2] = atan2(h->c[3], h->c[0])*r2d;
	if (RPY[2] < 0)
		RPY[2] = RPY[2] + 360;
	kpsd = (RPY[2] - e->GPShd) * d2r;




	kq[0] = - 0.5*(-h->q[1] * kpsn - h->q[2] * kpse - h->q[3] * kpsd);	/* quaternion */
	kq[1] = - 0.5*( h->q[0] * kpsn + h->q[3] * kpse - h->q[2] * kpsd);
	kq[2] = - 0.5*(-h->q[3] * kpsn + h->q[0] * kpse + h->q[1] * kpsd);
	kq[3] = - 0.5*( h->q[2] * kpsn - h->q[1] * kpse + h->q[0] * kpsd);

	for(i=0;i<3;i++){
		if (flag)
		{
			h->ba[i] = h->ba[i] + e->delx[9+i];
			h->bg[i] = h->bg[i] + e->delx[12+i];
		}
			h->q[i] = h->q[i] - kq[i];
	}
	h->q[3] = h->q[3] - kq[3]; // quaternion last element
	
	h->sts = sts_save; // recover process status
	return 1;
}

int		tframe_compen_mspd_sdins(type_mspd_sdins *m, type_at_epoch *e, int flag)
{
	// compensate e->delx to m
	// compensate medium speed nav varialbes 

	int i;
	int sts_save;
	double kq[4];
	double kpsn, kpse, kpsd;


	sts_save = m->sts;

	m->sts = -1; // assign highest priority

	m->lat = m->lat - e->delx[0];
	m->lon = m->lon - e->delx[1];
	m->hgt = m->hgt - e->delx[2];

	m->v[0] = m->v[0] - e->delx[3];
	m->v[1] = m->v[1] - e->delx[4];
	m->v[2] = m->v[2] - e->delx[5];


	kpsn = e->delx[6];						/* attitude */
	kpse = e->delx[7];
	kpsd = e->delx[8];


	kq[0] = - 0.5*(-m->q[1] * kpsn - m->q[2] * kpse - m->q[3] * kpsd);	/* quaternion */
	kq[1] = - 0.5*( m->q[0] * kpsn + m->q[3] * kpse - m->q[2] * kpsd);
	kq[2] = - 0.5*(-m->q[3] * kpsn + m->q[0] * kpse + m->q[1] * kpsd);
	kq[3] = - 0.5*( m->q[2] * kpsn - m->q[1] * kpse + m->q[0] * kpsd);


	for(i=0;i<3;i++){
		if (flag)
		{
			m->ba[i] = m->ba[i] + e->delx[9+i];
			m->bg[i] = m->bg[i] + e->delx[12+i];
		}
		m->q[i] = m->q[i] - kq[i];
		//e->lmdist[i]= e->lmdist[i] - e->delx[12+i];

	}
	m->q[3] = m->q[3] - kq[3]; // quaternion last element

	//e->vel_scale =  e->vel_scale-e->delx[15];

	m->sts = sts_save;	// recover process status
	return 1;
}

//double	round			(const double val)
//{
//	double a;
//	a = val + 0.5;
//	return floor(a);
//}
//
//void PrintT(const Matrix &m)
//{
//	printf("\nMatrix type: ", m.Type().Value());
//	printf("%d,%d)\n\n", m.Nrows(), m.Ncols());
//	if (m.IsZero()){
//		printf("All elements are zero\n");
//		return;
//	}
//	int nr=m.Nrows(); int nc=m.Ncols();
//	for (int i=1; i<=nr; i++)
//	{
//		for (int j=1; j<=nc; j++){
//			if((fabs(m(i,j))>1e10 || fabs(m(i,j))<1e-6) && (m(i,j)!=0))	printf("%15.7e\t",m(i,j));
//			else					printf("%15.7f\t",m(i,j));
//		}
//		printf("; %d\n", i);
//	}
//}
//
//Matrix	llh2xyz			(const Matrix& llh)
//{
//	//LLH2XYZ  Convert from latitude, longitude and height
//	//         to ECEF cartesian coordinates.  WGS-84
//	//
//	//  xyz  =  LLH2XYZ(llh)  
//	//
//	//  llh(1)  =  latitude in radians
//	//  llh(2)  =  longitude in radians
//	//  llh(3)  =  height above ellipsoid in meters
//	//
//	//  xyz(1)  =  ECEF x-coordinate in meters
//	//  xyz(2)  =  ECEF y-coordinate in meters
//	//  xyz(3)  =  ECEF z-coordinate in meters
//
//	//  Reference: Understanding GPS: Principles and Applications,
//	//             Elliott D. Kaplan, Editor, Artech House Publishers,
//	//             Boston, 1996.
//	//
//	//  M. & S. Braasch 10-96
//	//  Copyright (c) 1996 by GPSoft
//	//  All Rights Reserved.
//	double phi, lamb, h, sinphi, cosphi, coslam,  sinlam, tan2phi, tmp, tmpden, tmp2;
//	Matrix xyz(3,1);
//
//	phi  =  llh(1,1);
//	lamb  =  llh(2,1);
//	h  =  llh(3,1);
//	sinphi  =  sin(phi);
//	cosphi  =  cos(phi);
//	coslam  =  cos(lamb);
//	sinlam  =  sin(lamb);
//	tan2phi  =  pow(tan(phi),2.0);
//	tmp  =  1 - epe*epe;
//	tmpden  =  sqrt( 1 + tmp*tan2phi );
//	xyz(1,1)  =  (epam*coslam)/tmpden + h*coslam*cosphi;
//	xyz(2,1)  =  (epam*sinlam)/tmpden + h*sinlam*cosphi;
//	tmp2  =  sqrt(1 - epe*epe*sinphi*sinphi);
//	xyz(3,1)  =  (epam*tmp*sinphi)/tmp2 + h*sinphi;
//
//	return xyz;
//
//}



int		brm_avr_imu_pp(type_hspd_sdins *h, type_raw_imu **raw, type_avr_imu *avr, int idx, int flag)
{
	// bias remove and average imu data
	//
	// returns 1 if success
	// returns 0 if fail
	int i, j;
	double sum_fb[3];
	double sum_rbib[3];


	// shift & prepare imu data storage
	// the larger the index i, the older the data is
	for (i = ((int)(avr->msamp) - 1); i > 0; i--) {
		for (j = 0; j < 3; j++) {
			avr->p_fb[i][j] = avr->p_fb[i - 1][j];
			avr->p_rbib[i][j] = avr->p_rbib[i - 1][j];
		}
	}
	// first index i = 0 indicates the data just arrived,
	for (j = 0; j < 3; j++) {
		avr->p_fb[0][j] = raw[0]->fb[j] - h->ba[j];
		avr->p_rbib[0][j] = raw[0]->rbib[j] - h->bg[j];
	}
	// compute average data using the stored data
	for (j = 0; j < 3; j++) {
		sum_fb[j] = 0;
		sum_rbib[j] = 0;
	}
	for (j = 0; j < 3; j++) {
		for (i = 0; i < (avr->msamp); i++) {
			sum_fb[j] = sum_fb[j] + avr->p_fb[i][j];
			sum_rbib[j] = sum_rbib[j] + avr->p_rbib[i][j];
		}
	}
	for (j = 0; j < 3; j++) {
		if (flag)
		{
			avr->rbib[j] = sum_rbib[j] / (avr->msamp);
			avr->fb[j] = raw[0]->fb[j] - h->ba[j];
		}
		else
		{
			avr->rbib[j] = raw[0]->rbib[j];
			avr->fb[j] = raw[0]->fb[j];
		}
	}

	avr->sts = 1; // allow external use
	return 1;
}