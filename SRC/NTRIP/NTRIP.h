#pragma once

#include "NtripSocket.h"
#include "../NAV/ins.h"
#include "../NAV/Structure_type.h"


#define NBUFSIZE (32)
#define FILENAMESIZE (512)




////============================================================================
////
////	GNSS Receiver
////
////============================================================================
////--------------------------------------------------------------
////	UBX PVT
////--------------------------------------------------------------
typedef struct UBX_PVTMSG
{
	unsigned int iTOW;		// GPS time of week of the navigation epoch (ms)
	unsigned short year;	// Year (UTC) 
	unsigned char month;	// Month, range 1..12 (UTC)
	unsigned char day;		// Day of month, range 1..31 (UTC)
	unsigned char hour;		// Hour of day, range 0..23 (UTC)
	unsigned char minute;	// Minute of hour, range 0..59 (UTC)
	unsigned char second;	// Seconds of minute, range 0..60 (UTC)	
	unsigned char fixType;	// GNSSfix Type, range 0..5, 0:no-fix, 1:DR, 2:2D, 3:3D, 
	unsigned char flags;	// Fix Status Flags	
	unsigned char numSV;	// Number of satellites used in Nav Solution
	int lon;				// Longitude (deg), scaling: 1e-7
	int lat;				// Latitude (deg), scaling: 1e-7
	int height;				// Height above Ellipsoid (mm)
	int hMSL;				// Height above mean sea level (mm)
	unsigned int hAcc;		// Horizontal Accuracy Estimate (mm)
	unsigned int vAcc;		// Vertical Accuracy Estimate (mm)
	int velN;				// NED north velocity (mm/s)
	int velE;				// NED east velocity (mm/s)
	int velD;				// NED down velocity (mm/s)
	int gSpeed;				// Ground Speed (2-D) (mm/s)
	int headMot;			// Heading of motion 2-D (deg), scaling: 1e-5
	unsigned short pDOP;	// Position dilution of precision, scaling: 0.01
	int headingVehicle;		// Heading of vehicle, [deg], scaling: 1e-5
} PVTMSG, *PPVTMSG;

typedef struct UBX_PVTMSG2
{
	double iTOW;			// GPS time of week of the navigation epoch (ms)
	int year;				// Year (UTC) 
	int month;				// Month, range 1..12 (UTC)
	int day;				// Day of month, range 1..31 (UTC)
	int hour;				// Hour of day, range 0..23 (UTC)
	int minute;				// Minute of hour, range 0..59 (UTC)
	double second;			// Seconds of minute, range 0..60 (UTC)	
	unsigned char fixType;	// GNSSfix Type, range 0..5, 0:no-fix, 1:DR, 2:2D, 3:3D, 
	unsigned char flags;	// Fix Status Flags	
	unsigned char numSV;	// Number of satellites used in Nav Solution
	double lon;				// Longitude (deg), scaling: 1e-7
	double lat;				// Latitude (deg), scaling: 1e-7
	double height;			// Height above Ellipsoid (mm)
	double hMSL;			// Height above mean sea level (mm)
	double hAcc;			// Horizontal Accuracy Estimate (mm)
	double vAcc;			// Vertical Accuracy Estimate (mm)
	double velN;			// NED north velocity (mm/s)
	double velE;			// NED east velocity (mm/s)
	double velD;			// NED down velocity (mm/s)
	double gSpeed;			// Ground Speed (2-D) (mm/s)
	double headMot;			// Heading of motion 2-D (deg), scaling: 1e-5
	double pDOP;			// Position dilution of precision, scaling: 0.01
	double headingVehicle;	// Heading of vehicle, [deg], scaling: 1e-5
} PVTMSG2, *PPVTMSG2;




////============================================================================
////
////	EPSON G370N IMU Sensor
////
////============================================================================
////------------------------------------------------------------------
////	EPSON G370N IMU common
////------------------------------------------------------------------
#define G370N_CMD_LEN		(3)		// command packet length
#define G370N_RSP_LEN		(4)		// respond packet length
#define G370N_MAXBRST_LEN	(38)	// available maximun BURST data length
#define G370N_MAXBLOCK		(760)	// Availiable maxmum packet length = 38 bytes, 760=38*20
#define G370N_MAXBLOCK2		(G370N_MAXBLOCK+G370N_MAXBLOCK)	// 

#define G370N_RSP_WAIT_TIME	(20)	// mili-second


#define SF_TEMP (-0.0037918f)			// Temperature Scale Factor
#define SF_GYRO (0.0151515151515152f)	// Gyro Scale Factor
#define SF_ACCL (0.4f)					// Accl. Scale Factor

#define G370N_DELIMITER	(0x0D)	// end of packet identifier
#define G370N_CMD_RD	(0x00)	// command read


////------------------------------------------------------------------
////	EPSON G370N IMU Data Packet Struct
////------------------------------------------------------------------
typedef struct TAG_G370N_BRST
{
	BYTE ND;		// New Data flag
	BYTE EA;		// All Error flag

	float fTemp;	// [celcius]
	float fGyro[3];	// [deg/s] (dps)
	float fAccl[3];	// [mG]

	unsigned int cnt;		// Sensor internal ADC counter
	unsigned int checksum;	// checksum
} G370_BRST, *PG370_BRST;


////------------------------------------------------------------------
////	EPSON G370N IMU Operation Sequence/status
////------------------------------------------------------------------
#define G370N_STS_NONE			(0)	

#define G370N_STS_SOFTRST0		(1 << 0)
#define G370N_STS_NOTREADY0		(1 << 1)
#define G370N_STS_HARDERR00		(1 << 2)

#define G370N_STS_SMPLRATE		(1 << 8)
#define G370N_STS_FILTER		(1 << 9)
#define G370N_STS_BURST1		(1 << 10)
#define G370N_STS_BURST2		(1 << 11)

#define G370N_STS_BRSTPKTLEN	(1 << 12)

#define G370N_STS_UART_AUTO		(1 << 26)
#define G370N_STS_AUTO_START	(1 << 27)

#define G370N_STS_CONFIGMODE	(1 << 28)
#define G370N_STS_SMPLMODE		(1 << 29)

#define G370N_STS_RUNNING		(1 << 30)



////------------------------------------------------------------------
////	 EPSON G370N IMU Register Map (Address) & command
////	 - read : even address
////	 - write : even, odd address
////	 - NOTE: read command use **_L address (even)
////
////	 - NOTE: Description above about register address, read/write command is wrong. 
////			Register Map Address in datasheet is only for reading operation.
////			To write a command to a register, use address in example code in datasheet.
////			To read a register, use even address in the register map.
////	 - RR for Read Register, WR for Write Register, RV for Register Value
////------------------------------------------------------------------
//// BURST COMMAND REGISTER
#define G370N_BURST_CMD_L		(0x80)	// read command address


//// WINDOW CONTROL REGISTER
#define G370N_WR_WIN_CTRL		(0xFE)	// write command address
#define G370N_RV_WIN_0			(0x00)	// window 0
#define G370N_RV_WIN_1			(0x01)	// window 1
//
#define G370N_RR_WIN_CTRL_H		(0x7F)	// 
#define G370N_RR_WIN_CTRL_L		(0x7E)	// read command address


//// GLOBAL COMMAND REGISTER (window 1)
#define G370N_WR_GLOB_CMD		(0x8A)	// write command address
#define G370N_RV_SOFTRST		(0x80)	// do software reset
//
#define G370N_RR_GLOB_CMD_H		(0x0B)
#define G370N_RR_GLOB_CMD_L		(0x0A)	// read command address
#define G370N_RV_NOTREADY		(0x04)	// read bit


//// MODE CONTROL REGISTER (window 0)
#define G370N_WR_MODE_CTRL		(0x83)	// write command address
#define G370N_RV_MOVE_CONFIG	(0x02)	// move to configuration mode
#define G370N_RV_MOVE_SMPL		(0x01)	// move to sampling mode
//
#define G370N_RR_MODE_CTRL_H	(0x03)
#define G370N_RR_MODE_CTRL_L	(0x02)	// read command address
#define G370N_RV_MODE_STAT		(0x04)	// read bit


//// UART CONTROL REGISTER (window 1)
#define G370N_WR_UART_CTRL_H	(0x89)	// write command address
#define G370N_WR_UART_CTRL_L	(0x88)	// write command address
#define G370N_RV_UART_AUTO		(0x01)	// read bit & UART Automode ON
#define G370N_RV_AUTO_START		(0x02)	// read bit & Auto Start On
#define G370N_RV_UARTBR_460K	(0x00)	// 460800 bps
#define G370N_RV_UARTBR_230K	(0x01)	// 230400 bps
#define G370N_RV_UARTBR_921K	(0x02)	// 921600 bps
//
#define G370N_RR_UART_CTRL_H	(0x09)	//
#define G370N_RR_UART_CTRL_L	(0x08)	// read command address



//// DIAGNOSIS STATUS REGISTER (window 0)
#define G370N_RR_DIAG_STAT_H	(0x05)
#define G370N_RR_DIAG_STAT_L	(0x04)	// read command address
#define G370N_RV_HARD_ERR		(0x60)	// hard error bits


//// SAMPLING CONTROL REGISTER (window 1)
////	 - NOTE: see datasheet to select available sampling frequency
#define G370N_WR_SMPL_CTRL		(0x85)	// write command address
#define G370N_RR_SMPL_CTRL_H	(0x05)
#define G370N_RR_SMPL_CTRL_L	(0x04)	// read command address
#define G370N_RV_FS_2000		(0x00)	// sampling frequency 2000 Hz
#define G370N_RV_FS_1000		(0x01)
#define G370N_RV_FS_500			(0x02)
#define G370N_RV_FS_250			(0x03)
#define G370N_RV_FS_125			(0x04)
#define G370N_RV_FS_62			(0x05)
#define G370N_RV_FS_31			(0x06)
#define G370N_RV_FS_15			(0x07)
#define G370N_RV_FS_400			(0x08)
#define G370N_RV_FS_200			(0x09)
#define G370N_RV_FS_100			(0x0A)
#define G370N_RV_FS_80			(0x0B)
#define G370N_RV_FS_50			(0x0C)
#define G370N_RV_FS_40			(0x0D)
#define G370N_RV_FS_25			(0x0E)
#define G370N_RV_FS_20			(0x0F)	// sampling frequency 20 Hz


//// FILTER CONTROL REGISTER (window 1)
////	EPSON G370N IMU Filter Tap Size
////	 - MAF: Moving Average Filter
////	 - KAISER: Kaiser Filter
////	 - NOTE: see datasheet to select available fs
#define G370N_WR_FILT_CTRL		(0x86)	// write
#define G370N_RR_FILT_CTRL_H	(0x07)
#define G370N_RR_FILT_CTRL_L	(0x06)	// read
#define G370N_RV_MAF_TS_0		(0x00)	// zero tap
#define G370N_RV_MAF_TS_2		(0x01)	// Moving Average Filter, Tap Size = 2
#define G370N_RV_MAF_TS_4		(0x02)	//(0b00000010)
#define G370N_RV_MAF_TS_8		(0x03)	//(0b00000011)
#define G370N_RV_MAF_TS_16		(0x04)	//(0b00000100)
#define G370N_RV_MAF_TS_32		(0x05)	//(0b00000101)
#define G370N_RV_MAF_TS_64		(0x06)	//(0b00000110)
#define G370N_RV_MAF_TS_128		(0x07)	// Moving Average Filter, Tap Size = 128


//// BURST CONTROL REGISTER - 1 (window 1)
#define G370N_WR_BRST_CTRL1_H	(0x8D)	// write command address
#define G370N_WR_BRST_CTRL1_L	(0x8C)	// write command address
#define G370N_RR_BRST_CTRL1_H	(0x0D)	//
#define G370N_RR_BRST_CTRL1_L	(0x0C)	// read command address


//// BURST CONTROL REGISTER - 2 (window 1)
#define G370N_WR_BRST_CTRL2_H	(0x8F)	// write command address
#define G370N_RR_BRST_CTRL2_H	(0x0F)
#define G370N_RR_BRST_CTRL2_L	(0x0E)	// read command address




////============================================================================
////
////	Serial Port
////
////============================================================================
#define MAX_SERIAL_LEN	2560

// Flow control flags
#define FC_DTRDSR       0x01   //데이터 단말기(DTR) 대기,데이터 세트(DSR) 대기를 위한 신호
#define FC_RTSCTS       0x02
#define FC_XONXOFF      0x04

// ascii definitions
#define ASCII_BEL       0x07
#define ASCII_BS        0x08
#define ASCII_LF        0x0A
#define ASCII_CR        0x0D
#define ASCII_XON       0x11
#define ASCII_XOFF      0x13

class Port
{
public:

	////--------------------------------------------------------------
	////	PORT
	////--------------------------------------------------------------
	char port_name[20];		// port name
	HANDLE idComDev;		// port handel

	// port status
	BOOL fConnected;
	BOOL fXonXoff;
	BOOL fLocalEcho;
	BOOL fNewLine;
	BOOL fAutoWrap;
	BOOL fUseCNReceive;
	BOOL fDisplayErrors;
	BYTE bByteSize;
	BYTE bFlowCtrl;
	BYTE bParity;
	BYTE bStopBits;
	DWORD dwBaudRate;

	HANDLE	hWatchThread;	// thread handle
	DWORD	dwThreadID;	// thread ID

	// Overlapped I/O Struct
	OVERLAPPED osWrite;
	OVERLAPPED osRead;

	// member function
	//BOOL OpenPort_GPS(long port_number, long baud_rate_select = 6);
	BOOL OpenPort_GPS(long port_number, long baud_rate_select, void* pParent);
	//BOOL OpenPort_IMU(long port_number, long baud_rate_select = 6);
	BOOL OpenPort_IMU(long port_number, long baud_rate_select, void* pParent);

	BOOL SetupConnection(void);

	long rd_com(LPSTR lpszBlock, long nMaxLength);
	BOOL wt_com(LPSTR lpByte, DWORD dwBytesToWrite);

	BOOL ClosePort(void);


	////--------------------------------------------------------------
	////	GNSS
	////--------------------------------------------------------------
	unsigned char	rxbuff[MAX_SERIAL_LEN];
	FILE* m_fpGPS;


	////--------------------------------------------------------------
	////	IMU
	////--------------------------------------------------------------	
	unsigned int fstsimu;	// Status flag for IMU Operating Sequence
	G370_BRST pktimu;		// IMU Data Packet	
	FILE* m_fpIMU;			// file pointer to save IMU data
};




////============================================================================
////
////	class NTRIP
////
////============================================================================
#define WM_NTRIP_NOTICE		WM_APP+0x31		// NTRIP -> main

////------------------------------------------------------------------
////	options
////------------------------------------------------------------------
#define NTRIP_GPS			1
#define NTRIP_GLONASS		2

#define NTRIP_FILE_RECORDING	1

#define	REQ_SRCT				1
#define REQ_VRS					8

////------------------------------------------------------------------
////	SRCTable, Decoding, etc
////------------------------------------------------------------------
#define SRCT_HEAD		1
#define SRCT_CONTENTS	2
#define SRCT_FULL		4
#define SRCT_EMPTY		8

#define RES_EMPTY		1
#define RES_FULL		2
#define RES_FAIL		4

#define RTCM_VER20		1
#define RTCM_VER23		2
#define RTCM_VER31		4

#define CMR_VERN		1
#define CMR_VERP		2

#define MSG_RTCM		1
#define MSG_CMR			2

#define SRC_RTCM23		"RTCM2.3"
#define SRC_RTCM31		"RTCM 3"
#define SRC_CMR			"CMR"
#define SRC_CMRP		"CMR+"

#define MAXDATASIZE		2048	// 1.5 KBytes	// 너무 큰데....흠..
#define MAXNTRIPDATAS	200		
#define NUMSOURCE		32
#define NUM_DATA_TYPE	10

typedef struct tagType_STRTable 
{
	char	srcname[32];		// 
	char	mtpoint[32];		// Caster mountpoint
	char	id[32];				// Source identifier
	char	format[16];			// Data format
	int		msgformat, version;	// 
	char	formatd[16];		// Format details
	int		carrier;			// Data stream contains carrier phase information 0=NO, 1=Yes/L1, 2=Yes/L1&L2
	char	navsys[30];			// Navigation system
	char	network[20];		// Network
	char	country[4];			// Three character country code in ISO 3166
	double	latitude;			// Position, latitude, north
	double	longitude;			// Position, longitude, east
	int		nmea;				// Necessity for client to send NMEA message 0=must not send, 1=must send
	int		sol;				// Stream generated from 0=single base, 1=network
	char	gen[20];			// Hard- or software generating data stream
	char	compr[10];			// Compression/Encryption algorithm applied
	char	authen[2];			// Access protection for this particular data stream N=none, B=basic, D=digest
	char	fee[2];				// User fee for receiving this particular data stream N=no user fee, Y=usage is charged
	int		bitrate;			// Bit rate of data stream, bits per second
	char	misc[16];			// Miscellaneous information, last data field in record
	double	xyz[3];				// 
	double	enu[3];				// 
	char	usrname[16];		// 
	char	usrpass[16];		// 
} Type_STRTable, *LPType_STRTable;

typedef struct tagType_CASTable 
{
	char	host[129];		// Caster Internet host domain name or IP address
	int		port;			// Port number
	char	id[50];			// Caster identifier
	char	oper[10];		// Name of institution / agency / company operating the Caster
	int		nmea;			// Capability of Caster to receive NMEA message 0=not able to , 1=able to
	char	country[4];		// Three character country code in ISO 3166
	double	latitude;		// Position, latitude, north
	double	longitude;		// Position, longitude, east
	char	fbhost[129];	// Fallback Caster IP address
	int		fbport;			// Fallback Caster port number
	char	misc[20];		// Miscellaneous information, last data field in record
} Type_CASTable, *LPType_CASTable;

typedef struct tagType_NETTable 
{
	char	id[20];			// Network identifier
	char	oper[20];		// Name of institution / agency / company operating the Caster
	char	authen[2];		// Access protection for data streams of the network N=none, B=basic, D=digest
	char	fee[2];			// User fee for receiving this particular data stream N=no user fee, Y=usage is charged
	char	webnet[50];		// Web-address for network information
	char	webstr[50];		// Web-address for stream information
	char	webreg[50];		// Web address or mail address for registration
	char	misc[20];		// Miscellaneous information, last data field in record
} Type_NETTable, *LPType_NETTable;

typedef struct tagNTRIP_SOURCE_INFO 
{
	Type_STRTable	STR[NUMSOURCE];
	int				numSTR;
	Type_CASTable	CAS[10];
	int				numCAS;
	Type_NETTable	NET[10];
	int				numNET;
} NTRIP_SOURCE_INFO, *LPNTRIP_SOURCE_INFO;

typedef struct tagNTRIP_SRCTABLE_struct 
{
	char	data[NUMSOURCE+6][MAXNTRIPDATAS];
	char	lastdata[MAXNTRIPDATAS];
	int		status;
	int		rembyte;
	int		datalen;
	char	data2[MAXNTRIPDATAS];
	int		CONT_LEN;
	char	CONT[NUMSOURCE][MAXNTRIPDATAS];
	int		NTRIP_src_num;

	NTRIP_SOURCE_INFO SRCInfo;

} NTRIP_SRCTABLE_struct, *LPNTRIP_SRCTABLE_struct;

////	AUTHENTICATION - ICY 200 OK
typedef struct tagNTRIP_Response_struct 
{	
	int		status;
	int		datalen;
	char	data2[MAXNTRIPDATAS*5];
	int		CONT_LEN;
	char	CONT[MAXNTRIPDATAS];
} NTRIP_Response_struct, *LPNTRIP_Response_struct;

////	VRS 요청 위치
typedef struct tagARGVRS
{
	double	dLatd;		// latitude in degrees
	double	dLond;		// longitude in degrees	
	double	dAltm;		// altitude in meters
	
	char	RTKNet[NBUFSIZE];	// NGII : RTKNet-RTCM23, RTKNet-RTCM31, VRS-RTCM31, ...
} ARGVRS, *LPARGVRS;

typedef struct tagARGNTRIP
{	
	int			nID;	// 이 구조체의 아이디 = thread index
	
	//// owner에 접근할 방법 // owner라 함은 GUI 를 말하죠, 사용자에게 입력과 출력을 제시하는~!	
	HWND		hParent;		// 유저에게 무슨일들 하는지 실시간 공지가 있어야 하니께..
	HWND		hNetMonitor;	// Monitor Class에 메세지 전달할거거덩.
	DWORD		dwThreadID;		// NTRIP이 UIT에 종료하라고 메세지 날릴거거덩.??
	HANDLE		hThread;
	
	//// TCP/IP & ID/PW, etc
	char		szIP[NBUFSIZE];
	char		szPORT[NBUFSIZE];
	char		szID[NBUFSIZE];
	char		szPW[NBUFSIZE];
	LPNTRIP_SOURCE_INFO	pSRC;

	//// Port Number
	int			nPortGNSS;
	int			nPortIMU;
		
	//// Sampling Rate
	double		dFs_GNSS;
	double		dFs_IMU;
	double		dTapSize_IMU;

	//// options
	int			nGNSS;			// gps, glonass, sbas, etc
	int			nModeStream;	// on-line stream positioning vs file recording
	int			REQ_FLAG;		// obs, eph, vrs, nisl ...etc
	ARGVRS		VRS;			// vrs information

	//// status
	int isGNSSConnected;
	int isIMUConnected;
	int isNTRIPConnected;
	
	int engage_nav;
	int engage_gnss;


	////
	CListBox* pList;	
}ARGNTRIP, *LPARGNTRIP;



////============================================================================
////
////	class CNTRIP >> ntrip socket + gnss port + imu port + nav filter
////
////============================================================================

#define MAX_STRUCTSIZE	172800
#define MAX_LINE		256
#define MAX_LEAPSEC		17				//	maximum number of leapsecond array


#define IMU_STS_INIT_NOT_ENGAGE	0
#define IMU_STS_INIT_SUM		1
#define IMU_STS_IMU_READY		2
#define IMU_STS_GNSS_READY		4
#define IMU_STS_COARSE_ALIGN	8
#define IMU_STS_EKF				16

class CNTRIP : public CFrameWnd
{
	DECLARE_DYNCREATE(CNTRIP)
public:
	CNTRIP();
	virtual ~CNTRIP();
	
public:	
	CNTRIP(LPARGNTRIP pInfo);
	CNTRIP(LPNTRIP_SOURCE_INFO pSRC, LPARGNTRIP pInfo);	// GetSRCT 생성자
	
protected:
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnDestroy();


	////--------------------------------------------------------------
	////	Socket
	////--------------------------------------------------------------
public:
	CNtripSocket m_Sock;
	int CreateNtripSocket();
	void ReleaseNtripSocket();
	afx_msg LONG OnReceive(UINT wParam, LONG lParam);
	afx_msg LONG OnCloseServer(UINT wParam, LONG lParam);
	afx_msg LONG OnConnectError(UINT wParam, LONG lParam);
	afx_msg LONG OnConnectNoError(UINT wParam, LONG lParam);
	


	////--------------------------------------------------------------
	////	Source Table & Authentication
	////--------------------------------------------------------------	
public:
	LPNTRIP_SRCTABLE_struct	m_pSRCT;	// 메모리 할당해서 실제 수행에 사용
	LPNTRIP_SOURCE_INFO		m_pSrc;		// SRCTable 구한 후, 여기로 복사
	NTRIP_Response_struct	m_Res;
	int m_nIsAthenticated;
	
	int GetSourceTable();
	int InitSRCT(LPNTRIP_SOURCE_INFO pSRC);
	void Init_SRCTable_Info();
	void STRparser(char *data, Type_STRTable *str);
	void CASparser(char *data, Type_CASTable *cas);
	void NETparser(char *data, Type_NETTable *net);
	int Ntrip_SRC_Input_Proc(char rcvmsg);
	int ParseSourceTable();
	int encode(char *buf, int size, char *user, char *pwd);
	unsigned char checksum(char *buf, int size);
	int Ntrip_Response_Input_Proc(NTRIP_Response_struct *pkt, char rcvmsg);
	int NTRIPAthenticationProc(char *lpBuf, int nSize);

	
	////--------------------------------------------------------------
	////	etc
	////--------------------------------------------------------------	
public:	
	int m_nflagNtrip;	// -1: fail or reset, need to initializing befor use 
						//  1: success, use this class
	LPARGNTRIP m_pInfo;

	void SetInfo(LPARGNTRIP ntrip);
	int NtripInitializer(LPNTRIP_SOURCE_INFO pSRC, LPARGNTRIP pInfo);
	int NtripInitializer(LPARGNTRIP pInfo);
	int InitMPVRS();
	int GetRequestMsg(char *msg, int msglen, int& sendByte);
	void ReleaseAttribute();


	////--------------------------------------------------------------
	////	UITNtrip command
	////--------------------------------------------------------------	
public:
	int ConnectIMU();
	int ConnectGNSS();
	int ConnectNtrip();
	int NavStart();
	int NavStop();
	

	////--------------------------------------------------------------
	////	GNSS
	////--------------------------------------------------------------	
public:
	Port _serialGPS;

	double mf_iTOW;			// GPS time of week of the navigation epoch (ms)
	int mf_year;				// Year (UTC) 
	int mf_month;				// Month, range 1..12 (UTC)
	int mf_day;				// Day of month, range 1..31 (UTC)
	int mf_hour;				// Hour of day, range 0..23 (UTC)
	int mf_minute;				// Minute of hour, range 0..59 (UTC)
	double mf_second;			// Seconds of minute, range 0..60 (UTC)
	
	double mf_latd;
	double mf_lond;
	double mf_hgtm;
	
	double m_dfs_gnss;

	int m_is_GPS_available;	// 0(N/A), 1(OK)


	////--------------------------------------------------------------
	////	IMU
	////--------------------------------------------------------------	
public:
	Port _serialIMU;
	int isgpsupdated;
	int pre_gpst;
	int cur_gpst;

	double m_dfs_imu;







	////--------------------------------------------------------------
	////	GPS/INS EKF
	////--------------------------------------------------------------	


	FILE *fp_input;
	FILE* fp_inssol;
	FILE* fp_inssol2;

	char *input_file;
	int gps_read_cnt;
	int rmc_f, gga_f, vtg_f;
	int flag_gps_new, flag_init, ini_imu_flag;

	type_gps_sol			gps_sol;			// GPS 관련 구조체
	type_gps_sol			pre_gps_sol;		// GPS 관련 구조체
	type_raw_imu			**pprawimu;				// IMU/SPD 관련 구조체
	type_raw_imu			rawimu;

	type_hspd_sdins			h;					// 자세 관련 구조체
	type_mspd_sdins			m;					// 속도 관련 구조체
	type_at_epoch			pps;				// 필터 관련 구조체
	type_avr_imu			avr;				// 바이어스가 제거된 측정치

	int flag_gps, flag_ins;

	int Parser(char* stmp, char *tmp);
	int	GetGPSTfrmUTC(int year, int month, int day, int hour, int minute, double second, double *gpstime, int &leapsec);




	int Sync_idx_imu;
	double axyzsum[3];
	double rxyzsum[3];
	int num_calgn; // 초반 측정치 안정화를 위한 측정치 평균 범위 (GPS 초 단위)
	int ins_calgn;
	double tmpc, RPY[3];
	double ini_gpstime;
	




	int imu_sts;




	// 정지상태
	double acc_buf[2][100];
	double hacc[100];
	double hacc_std;
	double hacc_mean;
	double stop_sts;




public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	



	

	
};




