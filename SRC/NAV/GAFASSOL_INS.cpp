// GAFASSOL_INS.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include "GAFASSOL_INS.h"


// 광교 원천호수공원
//#define GNSSFILE	"DATA0\\log_GPS_2.txt"
//#define IMUFILE		"DATA0\\log_IMU_2.txt"
//#define NAVOUTFILE	"DATA0\\ins_sol.txt"

// 배곧생명공원-1
//#define GNSSFILE	"DATA1\\log_GPS_2.txt"
//#define IMUFILE		"DATA1\\log_IMU_2.txt"
//#define NAVOUTFILE	"DATA1\\ins_sol.txt"

// 배곧생명공원-2
//#define GNSSFILE	"DATA2\\log_GPS_route2.txt"
//#define IMUFILE		"DATA2\\log_IMU_route2.txt"
//#define NAVOUTFILE	"DATA2\\ins_sol.txt"


// 배곧생명공원 20210521 single antenna + spliter + dual rx - left
//#define GNSSFILE	"DATAdualL\\log_GPS.txt"
//#define IMUFILE		"DATAdualL\\log_IMU.txt"
//#define NAVOUTFILE	"DATAdualL\\ins_sol.txt"

// 배곧생명공원 20210521 single antenna + spliter + dual rx - right
#define GNSSFILE	"DATAdualR\\log_GPS.txt"
#define IMUFILE		"DATAdualR\\log_IMU.txt"
#define NAVOUTFILE	"DATAdualR\\ins_sol.txt"


int main()
{
	input_file = GNSSFILE;
	FILE *fp_inssol = fopen(NAVOUTFILE, "w");
	fclose(fp_inssol);

	int i, j, end_line,Sync_idx_imu;
	double axyzsum[3] = {0.0}, rxyzsum[3] = {0.0};
	int num_calgn = 20; // 초반 측정치 안정화를 위한 측정치 평균 범위 (GPS 초 단위)
	int ins_calgn = 1;
	double tmpc, RPY[3];
	double ini_gpstime = 0;

	/////////////////////////////////////////////////////////////////////////////////
	FILE* fIMU = fopen(IMUFILE, "r"); 

	int read_cnt = 0;		// temporary

	pprawimu = (type_raw_imu **) new type_raw_imu *[MAX_STRUCTSIZE];

	for (int k = 0; k < MAX_STRUCTSIZE; ++k)
		pprawimu[k] = (type_raw_imu *) new type_raw_imu;

	read_cnt = 0;
	while(!feof(fIMU) && read_cnt < MAX_STRUCTSIZE)
	{
		int flag_read = read_IMU(pprawimu[read_cnt++], fIMU);
		if ( flag_read == -1)	
			break;
		if (flag_read == 0)	
			read_cnt = read_cnt-1;

	}

	end_line = read_cnt-2;

	if (fIMU)	fclose(fIMU);

	printf("\n=========================================================\n");
	printf("NOTICE: read IMU data complete...");
	printf("\n=========================================================\n");

	/////////////////// initialization ///////////////////
	num_calgn = 100.0; // 초반 측정치 안정화를 위한 측정치 평균 범위
	ini_imu_flag = 1;
	for(Sync_idx_imu = 0;Sync_idx_imu < num_calgn;Sync_idx_imu++)
	{
		for(i=0;i<3;i++)
		{
			axyzsum[i] = axyzsum[i] + pprawimu[Sync_idx_imu]->fb[i];
			rxyzsum[i] = rxyzsum[i] + pprawimu[Sync_idx_imu]->rbib[i];
		}		
	}

	for(j=0;j<3;j++)
	{
		avr.fb[j] = axyzsum[j]/num_calgn; // 평균치 획득
		avr.rbib[j] = rxyzsum[j]/num_calgn;
	}
	/////////////////////////////////////////////////////////////////////////////////

	gps_read_cnt = 0;
	int ini_read;
	int ini_gps_f = 0;
	if ((fp_input = fopen(input_file, "r")) == NULL)
	{
		printf("Open file error!!\n");
	}
	else
	{
		while(!feof(fp_input))
		{
			int flag_read = 0;

			char stmp[MAX_LINE*2];

			fgets(stmp,sizeof(stmp),fp_input);

			//char szdelimiter[5] = ",";
			//char szdelimiter[5] = "\t";
			char *tmp;
			tmp = strtok(stmp, "\t");		// gps time
			gps_sol.gps_time = atof(tmp);

			while (flag_read == 0)
			{
				flag_read = Parser(stmp, tmp); 

				int proc_type = 0;

				if(Sync_idx_imu <= end_line)
				{
					double dIMUEPS = 0.001;

					if(pprawimu[Sync_idx_imu]->gps_time < ini_gpstime+1) 
					{
						while (1)
						{
							if (pprawimu[Sync_idx_imu]->gps_time >= ini_gpstime+1)
								break;
							else
								Sync_idx_imu++;
						}
					}
					if (ini_imu_flag == 0 )
					{
						while(pprawimu[Sync_idx_imu]->gps_time >= gps_sol.gps_time && pprawimu[Sync_idx_imu]->gps_time < gps_sol.gps_time+1)
						{
							if (pprawimu[Sync_idx_imu]->gps_time == gps_sol.gps_time)
							{
								/////////////////// SDINS Algorithm ///////////////////
								pps.gps_time = pprawimu[Sync_idx_imu]->gps_time;
								avr.gps_time = pprawimu[Sync_idx_imu]->gps_time;

								brm_avr_imu(&h, &pprawimu[Sync_idx_imu], &avr,Sync_idx_imu, 0); // 측정치 바이어스 제거

								do_hspd_sdins_in_rx_seq(&h, &m, &pps, &avr); // 자세계산
								// 자세계산 함수 내부에서 속도계산 함수(do_mspd_sdins_in_rx_seq_lvarm)를 호출함
								////////////////// SDINS Algorithm finish ///////////////////

								/////////////////// GPS_INS Integration ///////////////////
								///////////// <------------------------- GPS/INS sync 확인 후 GPS post solution 획득

								pps.gpsxyz[0] = gps_sol.ecefxyz[0];
								pps.gpsxyz[1] = gps_sol.ecefxyz[1];
								pps.gpsxyz[2] = gps_sol.ecefxyz[2];

								pps.lat = gps_sol.lat;
								pps.lon = gps_sol.lon;
								pps.hgt = gps_sol.hgt;

								pps.gpsvel[0] = gps_sol.vel[0];
								pps.gpsvel[1] = gps_sol.vel[1];
								pps.gpsvel[2] = gps_sol.vel[2];

								tframe_do_at_gps_tight_rx_in_rx_seq(&h, &m, &pps, &avr, &gps_sol, 1);  // GPS+INS 결합							

								gps_sol.delay = pps.delay;

								double tmpc, RPY[3], relpos[3];

								tmpc = 1 - pow(h.c[6], 2.0);
								tmpc = sqrt(tmpc);

								RPY[0] = atan2(h.c[7] , h.c[8]);
								RPY[1] = atan2(-h.c[6] , tmpc);
								RPY[2] = atan2(h.c[3] , h.c[0])*r2d;
								if (RPY[2] < 0)
									RPY[2] = RPY[2] + 360;

								
								if (0){

								}
								else{
									fp_inssol = fopen(NAVOUTFILE, "a+");

									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->gps_time);
									fprintf(fp_inssol," %20.10f",gps_sol.gps_time);

									fprintf(fp_inssol," %20.10f",RPY[0]*r2d); // 3
									fprintf(fp_inssol," %20.10f",RPY[1]*r2d);
									fprintf(fp_inssol," %20.10f",RPY[2]);

									fprintf(fp_inssol," %20.10f",m.lat*r2d); // 6
									fprintf(fp_inssol," %20.10f",m.lon*r2d);
									fprintf(fp_inssol," %20.10f",m.hgt);

									fprintf(fp_inssol," %20.10f",gps_sol.lat); // 9
									fprintf(fp_inssol," %20.10f",gps_sol.lon);
									fprintf(fp_inssol," %20.10f",gps_sol.hgt);
									fprintf(fp_inssol," %20.10f",gps_sol.hd);

									fprintf(fp_inssol," %20.10f",gps_sol.vel_horz);	//13

									fprintf(fp_inssol, "%20.10f", sqrt(m.v[0] * m.v[0] + m.v[1] * m.v[1]));

									/*fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[0]);	//14
									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[1]);	
									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[2]);	

									fprintf(fp_inssol," %20.10f",gps_sol.GDOP);				//17
									fprintf(fp_inssol," %20.10f",gps_sol.PDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.HDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.TDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.VDOP);

									fprintf(fp_inssol," %d",pps.m_up_type)*/;

									fprintf(fp_inssol," 0 \n");

									fclose(fp_inssol);
								}


								/////////////////// GPS_INS Integration finish ///////////////////
							}
							else
							{
								/////////////////// SDINS Algorithm ///////////////////
								pps.gps_time = pprawimu[Sync_idx_imu]->gps_time;
								avr.gps_time = pprawimu[Sync_idx_imu]->gps_time;

								brm_avr_imu(&h, &pprawimu[Sync_idx_imu], &avr,Sync_idx_imu, 0); // 측정치 바이어스 제거

								do_hspd_sdins_in_rx_seq(&h, &m, &pps, &avr); // 자세계산
								// 자세계산 함수 내부에서 속도계산 함수(do_mspd_sdins_in_rx_seq_lvarm)를 호출함
								////////////////// SDINS Algorithm finish ///////////////////

								double tmpc, RPY[3], relpos[3];

								tmpc = 1 - pow(h.c[6], 2.0);
								tmpc = sqrt(tmpc);

								RPY[0] = atan2(h.c[7] , h.c[8]);
								RPY[1] = atan2(-h.c[6] , tmpc);
								RPY[2] = atan2(h.c[3], h.c[0])*r2d;
								if (RPY[2] < 0)
									RPY[2] = RPY[2] + 360;

								if (0){

								}
								else{
									fp_inssol = fopen(NAVOUTFILE, "a+");

									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->gps_time);
									fprintf(fp_inssol," %20.10f",gps_sol.gps_time);

									fprintf(fp_inssol," %20.10f",RPY[0]*r2d); // 3
									fprintf(fp_inssol," %20.10f",RPY[1]*r2d);
									fprintf(fp_inssol," %20.10f",RPY[2]);

									fprintf(fp_inssol," %20.10f",m.lat*r2d); // 6
									fprintf(fp_inssol," %20.10f",m.lon*r2d);
									fprintf(fp_inssol," %20.10f",m.hgt);

									fprintf(fp_inssol," %20.10f",gps_sol.lat); // 9
									fprintf(fp_inssol," %20.10f",gps_sol.lon);
									fprintf(fp_inssol," %20.10f",gps_sol.hgt);
									fprintf(fp_inssol," %20.10f",gps_sol.hd);

									fprintf(fp_inssol," %20.10f",gps_sol.vel_horz);	//13
									fprintf(fp_inssol, "%20.10f", sqrt(m.v[0] * m.v[0] + m.v[1] * m.v[1]));

									/*fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[0]);	//14
									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[1]);	
									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[2]);	

									fprintf(fp_inssol," %20.10f",gps_sol.GDOP);				//17
									fprintf(fp_inssol," %20.10f",gps_sol.PDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.HDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.TDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.VDOP);

									fprintf(fp_inssol," %d",pps.m_up_type)*/;

									fprintf(fp_inssol," 1 \n");

									fclose(fp_inssol);
								}
							}
							Sync_idx_imu++;
						}
					}

					if(ini_imu_flag && gps_sol.vel_horz > 1)
					{
						nav_type_init_tight(&avr, &h, &m, &pps);		// SDINS 관련 변수 초기화
						course_align(&avr, &h, &m, &gps_sol);	// 초기 개략 정렬
						init_nav_by_lgps(gps_sol, m, h);			// GPS+INS 결합 관련 변수 초기화
						tframe_covar_tight(&pps);						// GPS+INS 결합 필터 관련 변수 초기화		//LimJH

						ini_gpstime = gps_sol.gps_time;
						gps_sol.delay = pps.delay;
						ini_imu_flag = 0;
						pps.gps_time_last_propa = ini_gpstime;
						
						/////////////////// initialization finish ///////////////////
					}
				}
				pre_gps_sol = gps_sol;
			}
		}
		printf ("Finished");
	}

	return 0;
}

int Parser(char* stmp, char *tmp)
{
	//char szdelimiter[5] = ",";
	//char szdelimiter[5] = "\t";
	tmp = strtok(NULL, "\t");

	if (tmp != NULL)
	{
		//gps_sol.gps_time = atof(tmp);	// 													//2
		tmp = strtok(NULL,",\t");															//3
		tmp = strtok(NULL,",\t");															//4
		tmp = strtok(NULL,",\t");															//5
		tmp = strtok(NULL,",\t");															//6
		tmp = strtok(NULL,",\t");															//7
		tmp = strtok(NULL,",\t");															//8
		tmp = strtok(NULL,",\t");															//9

		tmp = strtok(NULL,",\t");		gps_sol.lat = atof(tmp);		// Lat (deg)		//10
		tmp = strtok(NULL,",\t");		gps_sol.lon = atof(tmp);		// Lon (deg)		//11
		tmp = strtok(NULL,",\t");		gps_sol.hgt = atof(tmp);		// Hgt (m)			//12

		tmp = strtok(NULL,",\t");															//13							
		tmp = strtok(NULL,",\t");															//14
		tmp = strtok(NULL,",\t");															//15

		tmp = strtok(NULL, ",\t");		gps_sol.vel[0] = atof(tmp);							//16
		tmp = strtok(NULL, ",\t");		gps_sol.vel[1] = atof(tmp);							//17
		tmp = strtok(NULL, ",\t");		gps_sol.vel[2] = atof(tmp);							//18

		tmp = strtok(NULL, ",\t");		gps_sol.vel_horz = atof(tmp);						//19
		tmp = strtok(NULL, ",\t");		gps_sol.hd = atof(tmp);			// Heading(deg)		//20


		//tmp = strtok(NULL,"\t");										// DGPS Age(s)		//16
		//tmp = strtok(NULL,"\t");										// Sol type			//17
		//tmp = strtok(NULL,"\t");		gps_sol.hd = atof(tmp);			// Heading(deg)		//18
		//tmp = strtok(NULL,"\t");		gps_sol.vel[0] = atof(tmp);		// Vel.ECEF-X		//19
		//tmp = strtok(NULL,"\t");		gps_sol.vel[1] = atof(tmp);		// Vel.ECEF-Y		//20
		//tmp = strtok(NULL,"\t");		gps_sol.vel[2] = atof(tmp);		// Vel.ECEF-Z		//21
		//tmp = strtok(NULL,"\t");		gps_sol.vel_horz = atof(tmp);	// Vel.Horizontal	//22
		//tmp = strtok(NULL,"\t");		gps_sol.vel_vert = atof(tmp);	// Vel.Vertical		//23
		//tmp = strtok(NULL,"\t");										// Baseline Vec-X	//24
		//tmp = strtok(NULL,"\t");										// Baseline Vec-Y	//25
		//tmp = strtok(NULL,"\t");										// Baseline Vec-Z	//26
		//tmp = strtok(NULL,"\t");										// Baseline length	//27
		//tmp = strtok(NULL,"\t");										// Var.Baseline Vec-X		//28
		//tmp = strtok(NULL,"\t");										// Var.Baseline Vec-Y		//29
		//tmp = strtok(NULL,"\t");										// Var.Baseline Vec-Z		//30
		//tmp = strtok(NULL,"\t");										// Rover Clock bias			//31
		//tmp = strtok(NULL,"\t");										// Rover Clock bias rate	//32
	}
	
	return 1;
}

int		GetGPSTfrmUTC				(int year,int month,int day, int hour,int minute,double second,double *gpstime, int &leapsec)
{
	int   dayofw,dayofy, yr, ttlday, m, weekno;
	static  int  dinmth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	int i;

	//	leap second array
	const static int leapsec_arr[][7]={
		{2015,7,1,0,0,0,17},
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
	leapsec=0;
	for (yr=1981; yr<=(year-1); yr++)
	{
		ttlday  += 365;
		if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 ==0)
		{
			ttlday  += 1;
		}
	}
	ttlday += dayofy;
	weekno  = ttlday/7;
	dayofw  = ttlday - 7 * weekno;

	for(i=0;i<MAX_LEAPSEC;i++)
	{
		if((leapsec_arr[i][0]<=year))
		{
			if(leapsec_arr[i][1]<=month)
			{
				if(leapsec_arr[i][2]<=day)
				{
					leapsec = leapsec_arr[i][6];
					break;
				}
			}
		}
	}
	*gpstime =  (second + minute*60 + hour*3600 + dayofw*86400 + weekno*SECONDS_IN_WEEK + leapsec);

	return weekno;
}
