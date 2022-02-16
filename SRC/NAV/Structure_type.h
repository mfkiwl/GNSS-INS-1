#pragma once


#define AMAX			15					//	maximum dimension of P
#define SENAVRMAX		100					//	maximum sensor average interval count

///////////////////////////////////////////////////////////////////////////////////////////////////
//								GPS-Structure
///////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct  // type_gps_sol : gps solution by post processing
{
	int year, mon, day;
	int hour, min, sec;
	
	double gps_time;			  // gpstime
	double gps_time_last_propa;  // 전 시점 gpstime (코드 내부에서 사용, read 아님)

	double hd;	// gps 해딩

	double lat;	// 위도
	double lon;	// 경도
	double hgt;	// 고도
	double pre_hgt;
	double alt;

	double ecefxyz[3];	//위치

	double vel[3]; // ECEF frame 속도
	double vel_horz;	// 평면 속도
	double vel_vert;	// 수직방향 속도
	double vel_knot;

	double delay;	// gps/ins 시간오차 (코드 내부에서 사용, read 아님)

	//double lat_std;		//	m
	//double lon_std;		//	m
	//double hgt_std;		//	m

	double GDOP;
	double PDOP;
	double HDOP;
	double pre_HDOP;
	double TDOP;
	double VDOP;

	int numvis;
	int stat_RMC;
	int stat_GGA;
} type_gps_sol;


///////////////////////////////////////////////////////////////////////////////////////////////////
//								INS-Structure
///////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct  // type_raw_imu : raw imu data
{
	//double tag_time_acc;
	//double min_time_acc;
	//double pretag_time_acc;

	//double tag_time_gyro;
	//double gps_time; // from 2004/9/1 codes. gps_time becomes larger by considering weeknumber
	//double gps_stime; // gps_time of the first imu_data // should be initialized at once
	//double ce_time;
	//double cnt_frm_epoch;
	//double fb[3];
	//double rbib[3];
	//double spd;
	//double temperature;
	double gps_time; // from 2004/9/1 codes. gps_time becomes larger by considering weeknumber
	double gps_stime; // gps_time of the first imu_data // should be initialized at once
	double pgps_time;
	double ce_time;
	double cnt_frm_epoch;
	double fb[3];
	double rbib[3];

} type_raw_imu;

typedef struct // type_hspd_sdins :
//high speed SDINS data structure -> attitude computation rate
{
  /* note that the variables that should be initialized*/
  /* are commented with "init variable"                */

  int sts; // data status for external use
  // negative : some process other than main sdins process is updating this data.
  //        the lower its absolute value, the higher the priority of the current owner process is
    // 0 : under processing by main sdins process, thus no external read/write is permitted
    // 1 : processing is done. thus external read/write is permitted   

  double gps_time;// current gps time // from 2004/9/1 codes. gps_time becomes larger by considering weeknumber
  // indicates the leading time of imu data based on which current hspd is computed
  // the accompaning attitude info. is the status at  "gps_time + hspd_duration(usually 0.01sec)"
  double ce_time; // time marked by navigation computer internal clock
  double cnt_frm_epoch; // count from previous gps epoch(i.e, 
  double pcnt_frm_epoch;
  unsigned int idx_frm_epoch; // hspd index count from epoch(i.e. 0,1,2,...,99), should be reset to zero at PPS 
  int act; // action flag , just before the entry to hspd routine, one of the following was obtained 
        // 1 : gps data
        // 0 : imu data
        // -1: file end
        // -2: unexpected error(fmode = 1 )
  
  unsigned int rstcnt_to_mspd;  //$# INIT VARIABLE @%& // remaining count to next medium speed sdins navigation computation
  unsigned int rmcnt_to_mspd;   //$# INIT VARIABLE @%& // remaining count to next medium speed sdins navigation computation
  double dt;    //$# INIT VARIABLE @%&
          //* duration time of high speed sdins computation(0.1 or 0.01)
                    // dt should be initialized as proper value at start
  int integ_timing_reference; //$# INIT VARIABLE @%&
              // 0 if integration timing reference is dt
              // 1 if integration timing reference is gps time
  double q[4];  // the quaternion
  double c[9];  // C mtx from body to nav frame c[3]=c21, c[6]=c31
  double acc_fn[3];   //$# INIT VARIABLE @%& // accumulated specific force(ft/sec^2) w.r.t. nav frame
  unsigned int acc_fn_cnt;//$# INIT VARIABLE @%& //* specific force accumulation count //  should be initialized as 0 at start
  double dang[3];     //$# INIT VARIABLE @%& //* incremental angle during last imu meas. //  should be initialized as 0 at start

  // sensor biases used for this computation // at gps epoch these values change
  double ba[3]; //$# INIT VARIABLE @%& // accel bias used during this compuation
  double bg[3]; //$# INIT VARIABLE @%& // gyro drift used during this compuation

  // variables for the determination of zero-velocity condition
  double  no_zvelcond_cnt; // this count is incremented if total accel exceeds thresh hold ZVELACCEL
  double  max_mag_diff_accl; // maximum specific force difference magnitude from nominal g
  double  min_mag_diff_accl; // minimum specific force difference magnitude from nominal g
  double  accl_mag_sum;
  double  rot_rate_mag_sum;

  double  rpy[3]; // euler angle

  // updated at every hspd computation cycle
  // reset by every PPS signal

        // variables for next compuation cycle
  double pgps_time; //$# INIT VARIABLE=0 @%& // previous gps_time;
  double headd;
} type_hspd_sdins;

typedef struct // type_mspd_sdins
// medium speed SDINS data structure -> navigation computation rate
{
  /* note that the variables that should be initialized*/
  /* are commented with "INIT VARIABLE"                */

  int sts; // data status for external use
  // negative : some process other than main sdins process is updating this data.
  //        the lower its absolute value, the higher the priority of the current owner process is
    // 0 : under processing by main sdins process, thus no external read/write is permitted
    // 1 : processing is done. thus external read/write is permitted   

  double gps_time; // from 2004/9/1 codes. gps_time becomes larger by considering weeknumber
  // time at which current mspd is based on
  // compare with hspd where gps_time indicates not the hspd instant but the leading imu time
  double ce_time;
  double cnt_frm_epoch;
  double dt;    //$# INIT VARIABLE @%& 
          //* duration time of medium speed sdins computation
          // dt should be initialized at least at once
  int integ_timing_reference; //$# INIT VARIABLE @%&
              // 0 if integration timing reference is dt
              // 1 if integration timing reference is gps time

  // variable from high speed sdins computation
  double q[4];  //$ by course align // the quaternion
  double c[9];  //$ by course align // C mtx from body to nav frame
  double fn[3]; //$ derived INIT VARIABLE @%& //// estimated specific force(ft/sec^2) w.r.t. nav frame

    // variables during mid speed sdins computation 
  double lat0, lon0;  //$# INIT VARIABLE @%& //* position in (rad, rad, ft)
  double smlat, smlon; 
  double lat, lon, hgt;
  double v[3];      //$# INIT VARIABLE @%& //* velocity in ft/sec
  double dlat, dlon, dhgt; //$# INIT VARIABLE @%& //* d(pos)/dt
  double dv[3];            //$# INIT VARIABLE @%& //* dv/dt

  // sensor biases used for this computation
  double ba[3]; //$# INIT VARIABLE @%& // accel bias used during this compuation
  double bg[3]; //$# INIT VARIABLE @%& // gyro drift used during this compuation

   // variables for Kalman Filter System Matrix Compuation
  double slat, clat, slat2;//$ derived INIT VARIABLE @%& //
  double rnin[3], rnie[3], rnen[3], cor[3]; //$ derived INIT VARIABLE @%& //// estimated rotation rates of frames
  double g; //$# INIT VARIABLE @%& // estimated gravity
  double er;  //$# INIT VARIABLE @%& // earth rate in rad/sec // only once initialized at start     
  double rm;  //$ derived INIT VARIABLE @%& //// estimated earth radius of curvature in ft (north-south)
    double rt;  //$ derived INIT VARIABLE @%& //// estimated earth radius of curvature in ft (east-west)
  double rmh; //$ derived INIT VARIABLE @%& //// (rm+hgt)
  double rth; //$ derived INIT VARIABLE @%& //// (rt+hgt)

  // variables for next compuation cycle
  double pgps_time; //$# INIT VARIABLE=0 @%& // previous gps_time


} type_mspd_sdins;

typedef struct // type_at_epoch : low speed SDINS data structure -> filter computation rate
{
  /* note that the variables that should be initialized*/
  /* are commented with "INIT VARIABLE"                */

  int sts; // data status for external use
  // negative : some process other than main sdins process is updating this data.
  //        the lower its absolute value, the higher the priority of the current owner process is
    // 0 : under processing by main sdins process, thus no external read/write is permitted
    // 1 : processing is done. thus external read/write is permitted   

  int p_sts; //$# INIT VARIABLE=0@%& // covar matrix status
    // 0 : meas. update was not done
    // 1 : meas. update was done  

  //type_mspd_sdins msd; // navigation data used for this low speed sdins data

  double kvdt;    //$# INIT VARIABLE @%& //* duration time of velocity measurement update
  double kpdt;    //$# INIT VARIABLE @%& //* duration time of position measurement update

  double cotend; // course align finish time from the start of simulation
  double fitend; // fine align finish time from the start of simulation
  double natend; // navigation finish time from the start of simulation

  int rstcnt_to_p_up; // OPTION //$# INIT VARIABLE @%& // reset value : (int) kpdt/kvdt 
  int rmcnt_to_p_up;  // OPTION //$# INIT VARIABLE @%& // remaining count to next position-aided filter update
  double gps_time_last_propa; //$# INIT VARIABLE=0 @%& // time when last Kalman filter meas. update was achieved.
  double gps_time_last_meas_data;//$# INIT VARIABLE=0 @%& // time of the last meas. data for Kalman meas. update
    // Note : The above two time variables may be the same if no meas. delay is assumed.
    //        However, in the cases where meas. delay should be considered, both time marks should be different. 
  int m_up_type; //$# INIT VARIABLE=0 @%& // required measure update type
  // -1 for pure inertial mode(no external aid is required)
  // 0 for zero velocity update(when vehicle is not in movement)
  // 1 for gps vel. update only
  // 2 for gps pos. update only
  // 3 for gps pos & vel combined update
  // 4 for gps pos & zero vel combined update

  int n;        //$# INIT VARIABLE @%& // matrix order of the Kalman filter & error covar. mtx.
  double P[AMAX][AMAX]; //$# INIT VARIABLE @%& // the error covariance matrix
  double Q[AMAX];     //$# INIT VARIABLE @%& // white-noise strength matrix 
  double zvelstd;   //$# INIT VARIABLE @%& // zero velocity standard deviation in ft/sec
  double gpsvelstd; //$# INIT VARIABLE @%& // gps velocity standard deviation in ft/sec
  double phi[AMAX][AMAX]; //$# INIT VARIABLE @%& // state transition matrix from the previous epoch to the next epoch
  double delx[AMAX];    //$# INIT VARIABLE @%&=0 // Estimated state error to be compensated at next epoch(Kalman filter update time)
  

  /*                                                     */
  //======== mspd sdins information saved at PPS=========//
  /*                                                     */

  // negative : some process other than main sdins process is updating this data.
  //        the lower its absolute value, the higher the priority of the current owner process is
    // 0 : under processing by main sdins process, thus no external read/write is permitted
    // 1 : processing is done. thus external read/write is permitted   

  double gps_time; // from 2004/9/1 codes. gps_time becomes larger by considering weeknumber
  double ce_time;
  double cnt_frm_epoch;
  double dt;    //$# INIT VARIABLE @%& 
          //* duration time of medium speed sdins computation
          // dt should be initialized at least at once
  int integ_timing_reference; //$# INIT VARIABLE @%&
              // 0 if integration timing reference is dt
              // 1 if integration timing reference is gps time

  // variable from high speed sdins computation
  double q[4];  //$ by course align // the quaternion
  double c[9];  //$ by course align // C mtx from body to nav frame
  double fn[3]; //$ derived INIT VARIABLE @%& //// estimated specific force(ft/sec^2) w.r.t. nav frame

    // variables during mid speed sdins computation 
  double lat, lon, hgt; //$# INIT VARIABLE @%& //* position in (rad, rad, ft)
  double pre_lat, pre_lon, pre_hgt; //$# INIT VARIABLE @%& //* position in (rad, rad, ft)
  double lat0, lon0;  
  double smlat, smlon;
  double v[3];      //$# INIT VARIABLE @%& //* velocity in ft/sec
  double dlat, dlon, dhgt; //$# INIT VARIABLE @%& //* d(pos)/dt
  double dv[3];            //$# INIT VARIABLE @%& //* dv/dt

  // sensor biases used for this computation
  double ba[3]; //$# INIT VARIABLE @%& // accel bias used during this compuation
  double bg[3]; //$# INIT VARIABLE @%& // gyro drift used during this compuation

   // variables for Kalman Filter System Matrix Compuation
  double slat, clat, slat2;//$ derived INIT VARIABLE @%& //
  double rnin[3], rnie[3], rnen[3], cor[3]; //$ derived INIT VARIABLE @%& //// estimated rotation rates of frames
  double g; //$# INIT VARIABLE @%& // estimated gravity
  double er;  //$# INIT VARIABLE @%& // earth rate in rad/sec // only once initialized at start     
  double rm;  //$ derived INIT VARIABLE @%& //// estimated earth radius of curvature in ft (north-south)
    double rt;  //$ derived INIT VARIABLE @%& //// estimated earth radius of curvature in ft (east-west)
  double rmh; //$ derived INIT VARIABLE @%& //// (rm+hgt)
  double rth; //$ derived INIT VARIABLE @%& //// (rt+hgt)

  // OPTIONAL VARIABLES
  /*                                                     */
  //======== hspd sdins information saved at PPS=========//
  /*                                                     */
  double rbib[3]; // for lever arm correction
  double  no_zvelcond_cnt; // this count is incremented if total accel exceeds thresh hold ZVELACCEL
  double  accl_mag_sum; // for zero velocity determination
  double  rot_rate_mag_sum;// for zero velocity determination

  // variables for next compuation cycle
  double pgps_time; //$# INIT VARIABLE=0 @%& // previous gps_time
  double pdist; //2000

  /* For compressed measurements */
  int numcr; // number of CR
  int ibcr; // best cr index determined by average std

  // variables for tightly-coupled configuration
  double clockbias; // user clock bias
  double clockbrate; // scaled user clock bias rate

  // OPTIONAL VARIABLES : for feedforward configuration under federated filter architecture
  double x[AMAX];    //$# INIT VARIABLE @%&=0 // Accumulated state error to be compensated at next epoch(Kalman filter update time)
  double cdelx[AMAX];    //$# INIT VARIABLE @%&=0 // Estimated state error to be compensated at next epoch(Kalman filter update time)

  // variables for baro-aided INS
  double baro_m; // markov process
  double baro_s; // scale factor
  double gpsvel[3];
  double gpsxyz[3];
  double lmdist[3];
  double delay;
  double GPSrelhd;
  double GPShorvel;


  double kadt;
  double Pvalue[3];
  int stscnt;

  double GPShd;

  double pre_hd;
  int proc_cnt;
  double vel_scale;
  
  int rotGYRO_flag, pre_rotGYRO_flag;
  int pre_flag;

  double vel_obd[3];
} type_at_epoch; // sdins information after-gps-epoch processing

typedef struct // type_avr_imu : averaged imu data
{
	/* note that the variables that should be initialized*/
	/* are commented with "INIT VARIABLE"                */

	int sts; // data status for external use
	// negative : some process other than main sdins process is updating this data.
	//        the lower its absolute value, the higher the priority of the current owner process is
	// 0 : under processing by main sdins process, thus no external read/write is permitted
	// 1 : processing is done. thus external read/write is permitted   

	double gps_time; // from 2004/9/1 codes. gps_time becomes larger by considering weeknumber
	double ce_time;
	double cnt_frm_epoch;

	double msamp;   //$# INIT VARIABLE @%& // sampling number for imu average 

	double p_fb[SENAVRMAX][3];
	double p_rbib[SENAVRMAX][3];

	double fb[3];     // bias compensated & averaged accel meas.
	double rbib[3];   // bias compensated & averaged gyro meas.

	double ba[3];     //$# INIT VARIABLE @%& // accel bias used
	double bg[3];   //$# INIT VARIABLE @%& // gyro bias used

	double ca_roll;		// deg
	double ca_pitch;	// deg
	double ca_yaw;		// deg
} type_avr_imu;