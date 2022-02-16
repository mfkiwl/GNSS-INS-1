#pragma once
#include "../../stdafx.h"
#include "../../RTNetSolution.h"
//#include "../../RTNetSolutionDlg.h"
#include "NTRIP.h"
#include "../SUBFUNC/SUBFUNC.h"

//#pragma warning(disable: 4100)	// unreferenced variable
//#pragma warning(disable: 4101)	//참조되지 않은 지역 변수입니다.
//
//#pragma warning(disable: 4127)	// constant statement, if()
//#pragma warning(disable: 4146)	//단항 빼기 연산자가 부호 없는 형식에 적용되었습니다. 결과는 역시 unsigned입니다.
//#pragma warning(disable: 4189)	// local variable was initialized but unreferenced
//
//#pragma warning(disable: 4238)
#pragma warning(disable: 4244)	// casting - loss of data
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
#pragma warning(disable: 4996)	// deprecation


#define GNSS_PORT_NUM	(11)
#define IMU_PORT_NUM	(3)


IMPLEMENT_DYNCREATE(CNTRIP, CFrameWnd)

BEGIN_MESSAGE_MAP(CNTRIP, CFrameWnd)
	ON_MESSAGE(WM_RECEIVE_DATA, OnReceive)
	ON_MESSAGE(WM_CLOSE_SERVER, OnCloseServer)
	ON_MESSAGE(WM_CONNECT_ERROR, OnConnectError)
	ON_MESSAGE(WM_CONNECT_NOERROR, OnConnectNoError)
	ON_WM_DESTROY()
	ON_WM_TIMER()	
END_MESSAGE_MAP()


////============================================================================
////
////	Encoding Table	// NMEA message
////
////============================================================================
const static char encodingTable[64] = {
	'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
	'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
	'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
	'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};


////============================================================================
////
////	constructor, destructor, initializer, release buf, etc
////
////============================================================================
CNTRIP::CNTRIP()
{
}

CNTRIP::~CNTRIP()
{
	ReleaseAttribute();
}

CNTRIP::CNTRIP(LPARGNTRIP pInfo)
{	
}

CNTRIP::CNTRIP(LPNTRIP_SOURCE_INFO pSRC, LPARGNTRIP pInfo)
{		
}

int CNTRIP::NtripInitializer(LPARGNTRIP pInfo)
{	
	SetInfo(pInfo);	
	if ( InitMPVRS() == -1 )
	{
		m_nflagNtrip = -1;
		return -1;
	}	
	
	SetTimer(1, 10000, NULL);	// list box reset	

	_serialGPS.m_fpGPS = NULL;
	_serialGPS.m_fpIMU = NULL;
	_serialIMU.m_fpGPS = NULL;
	_serialIMU.m_fpIMU = NULL;

	_serialGPS.fConnected = 0;
	_serialIMU.fConnected = 0;


	//pprawimu = (type_raw_imu **) new type_raw_imu *[MAX_STRUCTSIZE];
	//for (int k = 0; k < MAX_STRUCTSIZE; ++k)
	//	pprawimu[k] = (type_raw_imu *) new type_raw_imu;


	mf_iTOW = 0;
	mf_year = 0;
	mf_month = 0;
	mf_day = 0;
	mf_hour = 0;
	mf_minute = 0;
	mf_second = 0;

	mf_latd = 0;
	mf_lond = 0;
	mf_hgtm = 0;

	m_dfs_gnss = pInfo->dFs_GNSS;
	m_dfs_imu = pInfo->dFs_IMU;

	m_is_GPS_available = 0;

	memset(&rawimu, 0, sizeof(type_raw_imu));
	
	fp_inssol = NULL;
	fp_inssol2 = NULL;
	
	/////////////////// initialization ///////////////////

	tmpc = 0;
	RPY[0] = .0;
	RPY[1] = .0;
	RPY[2] = .0;

	ini_gpstime = 0;

	num_calgn = 100.0; // 초반 측정치 안정화를 위한 측정치 평균 범위
	ins_calgn = 1;
	ini_imu_flag = 1;	// must be initialize to 1 (coarse alignment)
	
	Sync_idx_imu = 0;
	axyzsum[0] = .0;
	axyzsum[1] = .0;
	axyzsum[2] = .0;
	rxyzsum[0] = .0;
	rxyzsum[1] = .0;
	rxyzsum[2] = .0;
		
	imu_sts = IMU_STS_INIT_NOT_ENGAGE;


	/////////////////// initialization /////////////////// STOP
	stop_sts = -1;
	hacc_mean = 0;
	hacc_std = 0;
	for (int k = 0; k < 100; ++k)
	{
		acc_buf[0][k] = 0;
		hacc[k] = 0;
	}



	return 1;
}


int CNTRIP::NtripInitializer(LPNTRIP_SOURCE_INFO pSRC, LPARGNTRIP pInfo)
{
	ASSERT( pInfo->REQ_FLAG & REQ_SRCT );

	SetInfo(pInfo);
	if ( InitSRCT(pSRC) == -1 )
	{
		m_nflagNtrip = -1;
		return -1;
	}

	CString str = "NtripInitializer...SRCT";	
	m_pInfo->pList->AddString(str);
	m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	return 1;
}


int CNTRIP::CreateNtripSocket()
{
	m_Sock.SetHwnd( this->m_hWnd );
	if ( !m_Sock.Create())
		return -1;

	CString str = "CreateNtripSocket...";
	m_pInfo->pList->AddString(str.GetBuffer());
	m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	return 1;	
}


int CNTRIP::InitSRCT(LPNTRIP_SOURCE_INFO pSRC)
{
	if ( CreateNtripSocket() == FALSE )	return -1;

	m_pSrc = pSRC;
	if (pSRC == NULL)
		m_pSrc = new NTRIP_SOURCE_INFO;
	memset(m_pSrc, 0, sizeof(NTRIP_SOURCE_INFO));

	::ZeroMemory(m_pSrc, sizeof(NTRIP_SOURCE_INFO));

	m_pSRCT = (LPNTRIP_SRCTABLE_struct) new NTRIP_SRCTABLE_struct;
	::ZeroMemory(m_pSRCT, sizeof(NTRIP_SRCTABLE_struct));
	m_pSRCT->status = SRCT_EMPTY;	// SRCTable 이 비어있다고 초기화 해주는거임.	

	

	m_nIsAthenticated = -1;

	return 1;
}


int CNTRIP::InitMPVRS()
{
	if ( CreateNtripSocket() == FALSE )
		return -1;

	

	m_nIsAthenticated = -1;
	m_pSRCT = NULL;

	return 1;
}


void CNTRIP::SetInfo(LPARGNTRIP pInfo)
{
	m_pInfo = pInfo;
}


void CNTRIP::ReleaseAttribute()
{
	if ( m_pInfo->REQ_FLAG & REQ_SRCT && m_pSRCT != NULL )
	{
		delete m_pSRCT;
		m_pSRCT = NULL;
	}
		
	ReleaseNtripSocket();
	m_nflagNtrip = -1;

	//// 데이터 수집 테스트. 끝났는지 확인이 안되니까..여기서 박스하나 일단 띄운다. ...
	AfxMessageBox("simulation end ....");
}


void CNTRIP::ReleaseNtripSocket()
{
	m_Sock.ShutDown(2);	// send & receive
	//Sleep(100);
	m_Sock.Close();
}



////============================================================================
////
////	Get Source Table Input Procedure
////
////============================================================================
int CNTRIP::GetSourceTable()
{
	char buf[MAXDATASIZE] = { 0 };
	int recvByte = 0;

	recvByte = m_Sock.Receive(buf, MAXDATASIZE - 1);

	buf[recvByte] = 0;	// null terminator인데...필요할까..

	if (recvByte == 0)
	{
		return -1;	// server has been closed~!
	}
	else if (recvByte == SOCKET_ERROR)
	{
		DWORD dwErrorCode = GetLastError();
		if (dwErrorCode == WSAEWOULDBLOCK)
		{
			// TODO : no error
		}
		else
		{
			return -1;
		}
	}
	else if (recvByte > 0)
	{
		for (int i = 0; i < recvByte; ++i)
		{
			if (Ntrip_SRC_Input_Proc(buf[i]) == SRCT_FULL)
			{
				ParseSourceTable();
				memcpy(m_pSrc, &m_pSRCT->SRCInfo, sizeof(NTRIP_SOURCE_INFO));
				return 1;
			}//
		}// end for SRCT
	}

	return -1;
}

void CNTRIP::Init_SRCTable_Info()
{
	m_pSRCT->SRCInfo.numSTR = 0;
	m_pSRCT->SRCInfo.numCAS = 0;
	m_pSRCT->SRCInfo.numNET = 0;
}

int CNTRIP::Ntrip_SRC_Input_Proc(char rcvmsg)
{
	char *ptr;	

	switch (m_pSRCT->status)	
	{

	case SRCT_HEAD:
		m_pSRCT->data2[m_pSRCT->datalen] = rcvmsg;
		m_pSRCT->datalen++;
		if(m_pSRCT->datalen>1 && m_pSRCT->data2[m_pSRCT->datalen-1]=='\n')
		{
			if(m_pSRCT->data2[m_pSRCT->datalen-2]=='\r')
			{
				if(ptr = strstr(m_pSRCT->data2,"Server: "))
				{
				}
				else if(ptr = strstr(m_pSRCT->data2,"Content-Type: "))
				{
				}
				else if(ptr = strstr(m_pSRCT->data2,"Content-Length: "))
				{
				}
				else if(ptr = strstr(m_pSRCT->data2,"Date: "))
				{
				}
				else
				{
					if(strlen(m_pSRCT->data2)<=2)
					{
						m_pSRCT->CONT_LEN = 0;
						m_pSRCT->status = SRCT_CONTENTS;
					}
				}
				
				memset (m_pSRCT->data2, NULL, MAXNTRIPDATAS);
				m_pSRCT->datalen = 0;
			}
		}
		break;

	case SRCT_CONTENTS:
		m_pSRCT->data2[m_pSRCT->datalen] = rcvmsg;
		m_pSRCT->datalen++;
		if(m_pSRCT->datalen>1 && m_pSRCT->data2[m_pSRCT->datalen-1]=='\n')
		{
			if(m_pSRCT->data2[m_pSRCT->datalen-2]=='\r')
			{
				if(strstr(m_pSRCT->data2,"ENDSOURCETABLE"))
				{
					m_pSRCT->status = SRCT_FULL;			
				}
				else
				{
					m_pSRCT->data2[m_pSRCT->datalen] = '\0'; 				
					sprintf_s (m_pSRCT->CONT[m_pSRCT->CONT_LEN], MAXNTRIPDATAS - m_pSRCT->CONT_LEN, "%s", m_pSRCT->data2);
					m_pSRCT->CONT_LEN++;
				}				
				memset (m_pSRCT->data2, NULL, MAXNTRIPDATAS);
				m_pSRCT->datalen = 0;
			}
		}
		break;

	case SRCT_EMPTY:
		m_pSRCT->status = SRCT_EMPTY;
		m_pSRCT->data2[m_pSRCT->datalen] = rcvmsg;
		m_pSRCT->datalen++;
		if(m_pSRCT->datalen>1 && m_pSRCT->data2[m_pSRCT->datalen-1]=='\n')
		{
			if(m_pSRCT->data2[m_pSRCT->datalen-2]=='\r')
			{
				memset(m_pSRCT->data2,NULL,NUMSOURCE*MAXNTRIPDATAS);
				m_pSRCT->datalen = 0;
				m_pSRCT->status = SRCT_HEAD;
			}
		}
		break;

	case SRCT_FULL:
		break;
	}
	
	return m_pSRCT->status;
}



////============================================================================
////
////	Parse Source Table
////
////============================================================================
int CNTRIP::ParseSourceTable()
{
	int		i;

	Init_SRCTable_Info();

	m_pSRCT->NTRIP_src_num = m_pSRCT->CONT_LEN;

	for (i = 0; i < m_pSRCT->CONT_LEN; i++) 
	{
		if(toupper(m_pSRCT->CONT[i][0])=='C' && toupper(m_pSRCT->CONT[i][1])=='A' && toupper(m_pSRCT->CONT[i][2])=='S')
		{
			CASparser(m_pSRCT->CONT[i],&(m_pSRCT->SRCInfo.CAS[m_pSRCT->SRCInfo.numCAS]));
			m_pSRCT->SRCInfo.numCAS++;
		}
		else if(toupper(m_pSRCT->CONT[i][0])=='N' && toupper(m_pSRCT->CONT[i][1])=='E' && toupper(m_pSRCT->CONT[i][2])=='T')
		{
			NETparser(m_pSRCT->CONT[i],&(m_pSRCT->SRCInfo.NET[m_pSRCT->SRCInfo.numNET]));
			m_pSRCT->SRCInfo.numNET++;
		}
		else if(toupper(m_pSRCT->CONT[i][0])=='S' && toupper(m_pSRCT->CONT[i][1])=='T' && toupper(m_pSRCT->CONT[i][2])=='R')
		{
			STRparser(m_pSRCT->CONT[i],&(m_pSRCT->SRCInfo.STR[m_pSRCT->SRCInfo.numSTR]));
			m_pSRCT->SRCInfo.numSTR++;
		}
	}
	return 0;
}


void CNTRIP::STRparser(char *data, Type_STRTable *str)
{
	int j = 1;
	char *p, strtmp[100];
	char *nextToken = NULL;

	p = strtok_s (data, ";", &nextToken);

	while (1)
	{	
		p = strtok_s (NULL, ";", &nextToken);
		if (p == NULL)
			break;

		sprintf_s (strtmp, _countof(strtmp), "%s", p);

		j++;
		switch (j)
		{

		case 2:
			sprintf_s (str->srcname, _countof(str->srcname), strtmp);
			sprintf_s (str->mtpoint, _countof(str->mtpoint), strtmp);
			break;

		case 3:
			sprintf_s (str->id, _countof(str->id), strtmp);
			break;

		case 4:
			sprintf_s(str->format, _countof(str->format), strtmp);			
			if(strstr(strtmp,"RTCM"))
			{
				str->msgformat = MSG_RTCM;	  str->version = 0;
				if(strstr(strtmp,"2.0"))	  str->version = RTCM_VER20;
				else if(strstr(strtmp,"2.3")) str->version = RTCM_VER23;
				else if(strstr(strtmp,"3.1")) str->version = RTCM_VER31;
				else if(!strstr(strtmp,"2.3") && strstr(strtmp,"3")) str->version = RTCM_VER31;
				else if(!strstr(strtmp,"2.0") && !strstr(strtmp,"2.3") && strstr(strtmp,"2")) str->version = RTCM_VER20;
			}
			else if(strstr(strtmp,"CMR"))
			{
				str->msgformat = MSG_CMR;	  str->version = CMR_VERN;
				if(strstr(strtmp,"+"))		  str->version = CMR_VERP;
			}
			else
			{
				str->msgformat = 0;			  str->version = 0;
			}
			break;

		case 5:		sprintf(str->formatd,strtmp);	break;
		case 6:		str->carrier = atoi(strtmp);	break;
		case 7:		sprintf(str->navsys,strtmp);	break;
		case 8:		sprintf(str->network,strtmp);	break;
		case 9:		sprintf(str->country,strtmp);	break;
		case 10:	str->latitude = atof(strtmp);	break;
		case 11:	str->longitude = atof(strtmp);	break;
		case 12:	str->nmea = atoi(strtmp);		break;
		case 13:	str->sol = atoi(strtmp);		break;
		case 14:	sprintf(str->gen,strtmp);		break;
		case 15:	sprintf(str->compr,strtmp);		break;
		case 16:	sprintf(str->authen,strtmp);	break;
		case 17:	sprintf(str->fee,strtmp);		break;
		case 18:	str->bitrate = atoi(strtmp);	break;
		default:	sprintf(str->misc,strtmp);		break;
		}
	}
}


void CNTRIP::NETparser(char *data, Type_NETTable *net)
{
	int j;
	char *p, strtmp[100];
	p = strtok(data,";");
	j=1;
	
	while(1)
	{
		p = strtok(NULL,";");	sprintf(strtmp,"%s",p);
		if(p == NULL)
			break;
		
		j++;
		switch(j)
		{
		case 2:	sprintf(net->id,strtmp);	break;
		case 3:	sprintf(net->oper,strtmp);	break;
		case 4:	sprintf(net->authen,strtmp);break;
		case 5:	sprintf(net->fee,strtmp);	break;
		case 6:	sprintf(net->webnet,strtmp);break;
		case 7:	sprintf(net->webstr,strtmp);break;
		case 8:	sprintf(net->webreg,strtmp);break;
		case 9:	sprintf(net->misc,strtmp);	break;
		default:							break;
		}
	}
}


void CNTRIP::CASparser(char *data, Type_CASTable *cas)
{
	int j;
	char *p, strtmp[100];
	char *nextToken = NULL;

	p = strtok_s (data, ";", &nextToken);	// p = "CAS";
	j=1;

	while(1)
	{
		p = strtok_s (NULL, ";", &nextToken);
		sprintf(strtmp,"%s",p);
		if(p == NULL)
			break;
		j++;

		switch(j)
		{
		case 2:		sprintf(cas->host,strtmp);		break;
		case 3:		cas->port = atoi(strtmp); 		break;
		case 4:		sprintf(cas->id,strtmp);		break;
		case 5:		sprintf(cas->oper,strtmp);		break;
		case 6:		cas->nmea = atoi(strtmp);		break;
		case 7:		sprintf(cas->country,strtmp);	break;
		case 8:		cas->latitude = atof(strtmp);	break;
		case 9:		cas->longitude = atof(strtmp);	break;
		case 10:	sprintf(cas->fbhost,strtmp);	break;
		case 11:	sprintf(cas->misc,strtmp);		break;
		default:									break;
		}
	}
}



////============================================================================
////
////	Response Input Procedure
////		- ICY 200 OK
////
////============================================================================
int CNTRIP::NTRIPAthenticationProc(char *lpBuf, int nSize)
{
	int j = 0;

	while (m_Res.status != RES_FULL)
	{
		int status = Ntrip_Response_Input_Proc(&m_Res, lpBuf[j++]);
		if (status == RES_EMPTY)
			continue;
		else if (status == RES_FULL)
		{
			m_nIsAthenticated = 1;
			return 1;
		}
		else if (status == RES_FAIL)
		{
			m_nIsAthenticated = -1;
			return -1;
		}
		else
		{
			m_nIsAthenticated = -1;
			return -1;
		}

		if (j >= nSize - 1)
		{
			m_nIsAthenticated = -1;
			return -1;
		}
	}// end while (NTRIP_Resp.status != RES_FULL) {	

	m_nIsAthenticated = 0;
	return 0;	// 1:ok, -1:denied, 0:not need to.
}

int CNTRIP::Ntrip_Response_Input_Proc(NTRIP_Response_struct *pkt, char rcvmsg)
{
	switch(pkt->status)
	{
	case RES_FULL:
		break;
	case RES_EMPTY:
	default:
		pkt->status = RES_EMPTY;
		pkt->data2[pkt->datalen] = rcvmsg;
		pkt->datalen++;//TRACE("%s\n",pkt->data2);
		if(pkt->datalen>3 && pkt->data2[pkt->datalen-1]=='\n')
		{
			if(pkt->data2[pkt->datalen-2]=='\r')
			{
				if(strstr(pkt->data2,"ICY 200 OK"))
				{
					CString str = "ICY 200 OK";
					m_pInfo->pList->AddString(str);
					m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
					sprintf(pkt->CONT,"%s",pkt->data2);
					pkt->status = RES_FULL;
				}
				else
				{
					sprintf(pkt->CONT,"%s",pkt->data2);
					pkt->status = RES_FAIL;
				}
				memset(pkt->data2,NULL,MAXNTRIPDATAS*5);
				pkt->datalen = 0;
			}
		}
		break;
	}
	
	return pkt->status;	
}


////============================================================================
////
////	Encoding & Parity Check
////
////============================================================================
int CNTRIP::encode(char *buf, int size, char *user, char *pwd)
{
	unsigned char inbuf[3];
	char *out = buf;
	int i, sep = 0, fill = 0, bytes = 0;

	while (*user || *pwd) 
	{
		i = 0;
		while (i < 3 && *user)
			inbuf[i++] = *(user++);

		if (i < 3 && !sep) 
		{
			inbuf[i++] = ':';
			++sep;
		}

		while (i < 3 && *pwd)
			inbuf[i++] = *(pwd++);

		while (i < 3)
		{
			inbuf[i++] = 0;
			++fill;
		}

		if (out - buf < size-1)
			*(out++) = encodingTable[(inbuf[0] & 0xFC) >> 2];

		if (out - buf < size-1)
			*(out++) = encodingTable[ ((inbuf[0] & 0x03) << 4) | ((inbuf[1] & 0xF0) >> 4) ];

		if (out - buf < size-1) 
		{
			if (fill == 2)
				*(out++) = '=';
			else
				*(out++) = encodingTable[ ((inbuf[1] & 0x0F) << 2) | ((inbuf[2] & 0xC0) >> 6) ];
		}

		if (out - buf < size-1) 
		{
			if (fill >= 1)
				*(out++) = '=';
			else
				*(out++) = encodingTable[ inbuf[2] & 0x3F];
		}

		bytes += 4;
	} // while (*user || *pwd)

	if (out - buf < size)
		*out = 0;

	return bytes;
}


unsigned char CNTRIP::checksum(char *buf, int size)
{
	unsigned char check=0;
	int i;
	for (i = 0; i < size; i++)
	{
		check ^= buf[i];
	}
	return check;
}


////============================================================================
////
////	Network Access & process something
////
////============================================================================
int CNTRIP::GetRequestMsg(char *msg, int msglen, int& sendByte)
{		
	//	sendByte = _snprintf_s( msg, msglen, msglen - 40,
	//		"GET /0 HTTP/1.1\r\nUser-Agent: NTRIP GnssSurferV1.07a\r\nAuthorization: Basic " );
			
	if (m_pInfo->REQ_FLAG & REQ_SRCT) {// NDGPS는 요청메세지 다릅니다. 작성하세요~!
		sendByte = _snprintf_s( msg, msglen, msglen - 40,
			"GET /0 HTTP/1.1\r\nUser-Agent: NTRIP GnssSurferV1.07a\r\nAuthorization: Basic " );
		sendByte += encode(msg + sendByte, msglen - sendByte - 5, m_pInfo->szID, m_pInfo->szPW);		
		_snprintf_s(msg + sendByte, msglen - sendByte,  4, "\r\n\r\n");
		sendByte += 4;
	}

	//else if (m_pInfo->REQ_FLAG & REQ_MP){ // Get RTCM data, exclude VRS					
	//	sendByte = _snprintf_s( msg, msglen, msglen - 40,
	//		"GET /%s HTTP/1.1\r\nUser-Agent: NTRIP GnssSurferV1.07a\r\nAuthorization: Basic ",
	//		m_pInfo->MPt);
	//	sendByte += encode(msg + sendByte, msglen - sendByte - 5, m_pInfo->szID, m_pInfo->szPW);		
	//	_snprintf_s(msg + sendByte, msglen - sendByte,  4, "\r\n\r\n");
	//	sendByte += 4;
	//}

	else if ( m_pInfo->REQ_FLAG & REQ_VRS ){ // NDGPS는 요청메세지 다릅니다. 작성하세요~!
		char	bufsend[256] = {0};		// send msg
		char	buftmp[256] = {0};		// for vrs request msg		

		sendByte = _snprintf_s( bufsend, _countof(bufsend), _countof(bufsend) - 40,
			"GET /%s HTTP/1.1\r\nUser-Agent: NTRIP GnssSurferV1.07a\r\nAuthorization: Basic ",
			m_pInfo->VRS.RTKNet);
		sendByte += encode(bufsend + sendByte, _countof(bufsend) - sendByte - 5, m_pInfo->szID, m_pInfo->szPW);		
		_snprintf_s(bufsend + sendByte, _countof(bufsend) - sendByte,  4, "\r\n\r\n");
		sendByte += 4;

		if (mf_latd != 0 && mf_lond != 0)
		{
			m_pInfo->VRS.dLatd = mf_latd;
			m_pInfo->VRS.dLond = mf_lond;
			m_pInfo->VRS.dAltm = mf_hgtm;
		}		

		double nmealat = DegreeToNMEA0183(m_pInfo->VRS.dLatd);
		double nmealon = DegreeToNMEA0183(m_pInfo->VRS.dLond);
		double nmeaalt = m_pInfo->VRS.dAltm;
		struct tm t;	GetUTC(&t);		
				
		_snprintf_s( buftmp, _countof(buftmp), 128,
			"GPGGA,%02d%02d%02d,%9.4f,N,%10.4f,E,1,05,1.00,+00100,M,%06.3f,M,,",
			t.tm_hour, t.tm_min, t.tm_sec, nmealat, nmealon, nmeaalt);

		_snprintf_s(msg, msglen, msglen, "%s$%s*%02x\r\n",bufsend, buftmp, checksum(buftmp, (int)strlen(buftmp)) );
		sendByte = (int)strlen(msg);		
	}
	else
		;	

	CString str = msg;
	m_pInfo->pList->AddString(str);
	m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	return 1;
}

int CNTRIP::ConnectIMU()
{
	int ret1 = 0;
	//if (ret1 = _serialIMU.OpenPort_IMU(IMU_PORT_NUM, 460800, this))
	if (ret1 = _serialIMU.OpenPort_IMU(m_pInfo->nPortIMU, 460800, this))
	{
		unsigned char cmd[G370N_CMD_LEN] = { 0 };
		int cnt_sts_wait = 0;
		Sleep(1000);	// wait 1000 ms


		////--------------------------------------------------------------
		////	1. Software Reset (UART)
		////--------------------------------------------------------------
		TRACE("1. Software Reset (UART)\n");

		// WINDOW = 1
		TRACE(">> WT : WIN_CTRL - WINDOW = 1\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_1;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		// software reset
		TRACE(">> WT : GLOB_CMD_L - SOFT_RST\n");
		//cmd[0] = G370N_RR_GLOB_CMD_L;
		cmd[0] = G370N_WR_GLOB_CMD;
		cmd[1] = G370N_RV_SOFTRST;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(1600);	// wait 1600 ms

		TRACE(">> RD : GLOB_CMD_L\n");
		cmd[0] = G370N_RR_GLOB_CMD_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE("\n\n\n");


		////--------------------------------------------------------------
		////	2. Configuration Mode
		////--------------------------------------------------------------
		TRACE("2. Configuration Mode\n");

		// WINDOW = 0
		TRACE(">> WT : WIN_CTRL - WINDOW = 0\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_0;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);



		// move to configuration mode
		TRACE(">> WT : MODE_CTRL_- Move to Configuration Mode\n");
		cmd[0] = G370N_WR_MODE_CTRL;
		cmd[1] = G370N_RV_MOVE_CONFIG;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : MODE_CTRL_L\n");
		cmd[0] = G370N_RR_MODE_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);


		TRACE("\n\n\n");



		////--------------------------------------------------------------
		////	3. UART Auto Mode ON / AUTO Start Off
		////--------------------------------------------------------------
		TRACE("3. UART Auto Mode ON / AUTO Start Off\n");

		// WINDOW = 1	
		TRACE(">> WT : WIN_CTRL - WINDOW = 1\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_1;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);


		// UART Auto Mode on, AUTO Start off
		TRACE(">> WT : UART_CTRL_L - UART Auto Mode on, AUTO Start off = 1\n");
		cmd[0] = G370N_WR_UART_CTRL_L;
		cmd[1] = G370N_RV_UART_AUTO;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : UART_CTRL_L\n");
		cmd[0] = G370N_RR_UART_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE("\n\n\n");



		////--------------------------------------------------------------
		////	4. Power-on sequence (UART)
		////	
		////		NOTE: Register address of WIN_CTRL in Example code is correct
		////			- Register Map : 0x7F, 0x7E
		////			- Example Code : 0xFE
		////			
		////--------------------------------------------------------------
		TRACE("4. Power-on sequence (UART)\n");

		// 1.1. check NOT_READY bit goes to 0.	
		// WINDOW = 1	
		TRACE(">> WT : WIN_CTRL - WINDOW = 1\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_1;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		// GLOB_CMD read command
		Sleep(300);
		//cnt_sts_wait = 0;
		//while (!(_serial.fsts & G370N_STS_NOTREADY0))
		//{		
		//	printf(">> RD : GLOB_CMD_L (%d)\n", cnt_sts_wait + 1);
		//	cmd[0] = G370N_RR_GLOB_CMD_L;
		//	cmd[1] = G370N_CMD_RD;
		//	cmd[2] = G370N_DELIMITER;
		//	_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//	++cnt_sts_wait;		
		//	Sleep(G370N_RSP_WAIT_TIME);
		//	if (cnt_sts_wait > 10)
		//	{
		//		printf("\tERROR >> IMU STS : GLOB_CMD_L - NOT_READY failure...\n");
		//		return 1;
		//	}
		//	Sleep(300);
		//}
		//printf("\t>> IMU STS : GLOB_CMD_L - NOT_READY = 0\n\n\n");


		// 1.2. confirm HARD_ERR bits.
		// WINDOW = 0
		TRACE(">> WT : WIN_CTRL - WINDOW = 0\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_0;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		// DIAG_STAT read command
		Sleep(300);
		//cnt_sts_wait = 0;
		//while (!(_serial.fsts & G370N_STS_HARDERR00))
		//{
		//	printf(">> RD : DIAG_STAT_L (%d)\n", cnt_sts_wait + 1);
		//	cmd[0] = G370N_RR_DIAG_STAT_L;
		//	cmd[1] = G370N_CMD_RD;
		//	cmd[2] = G370N_DELIMITER;
		//	_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//	++cnt_sts_wait;		
		//	Sleep(G370N_RSP_WAIT_TIME);
		//	if (cnt_sts_wait > 10)
		//	{
		//		printf("\tERROR >> IMU STS : DIAG_STAT_L - HARD_ERR failure ...\n");
		//		return 1;
		//	}
		//	Sleep(300);
		//}
		//printf("\t>> IMU STS : DIAG_STAT_L - HARD_ERR = 00\n\n\n");

		TRACE("\n\n\n");



		////--------------------------------------------------------------
		////	5. UART auto mode (16 bits burst data) Sampling Frequency and Filter Tap Size
		////--------------------------------------------------------------
		TRACE("5. UART auto mode (16 bits burst data) Sampling Frequency and Filter Tap Size\n");

		// WINDOW = 1
		TRACE(">> WT : WIN_CTRL - WINDOW = 1\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_1;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);


		////--------------------------------------------------------------
		////	option 0: Fs = 125, MAF TS = 16
		////--------------------------------------------------------------
		//// Sampling Frequency
		//printf(">> WT : SMPL_CTRL_L - Fs=20\n");
		//cmd[0] = G370N_WR_SMPL_CTRL;
		//cmd[1] = G370N_RV_FS_125;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//printf(">> RD : SMPL_CTRL_L\n");
		//cmd[0] = G370N_RR_SMPL_CTRL_L;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//// Moving Average Filter, Tap Size = 64
		//printf(">> WT : FILT_CTRL_L - Tap size=64\n");
		//cmd[0] = G370N_WR_FILT_CTRL;
		//cmd[1] = G370N_RV_MAF_TS_16;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//printf(">> RD : FILT_CTRL_L\n");
		//cmd[0] = G370N_RR_FILT_CTRL_L;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		////--------------------------------------------------------------
		////	option 1: Fs = 20, MAF TS = 64
		////--------------------------------------------------------------
		//// Sampling Frequency
		//TRACE(">> WT : SMPL_CTRL_L - Fs=20\n");
		//cmd[0] = G370N_WR_SMPL_CTRL;
		//cmd[1] = G370N_RV_FS_20;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//TRACE(">> RD : SMPL_CTRL_L\n");
		//cmd[0] = G370N_RR_SMPL_CTRL_L;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//// Moving Average Filter, Tap Size = 64
		//TRACE(">> WT : FILT_CTRL_L - Tap size=64\n");
		//cmd[0] = G370N_WR_FILT_CTRL;
		//cmd[1] = G370N_RV_MAF_TS_64;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//TRACE(">> RD : FILT_CTRL_L\n");
		//cmd[0] = G370N_RR_FILT_CTRL_L;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		////--------------------------------------------------------------
		//// option 2: Fs = 40, MAF TS = 64
		////--------------------------------------------------------------
		//// Sampling Frequency
		//cmd[0] = G370N_RR_SMPL_CTRL_H;	cmd[1] = G370N_FS_40;	cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);	Sleep(G370N_RSP_WAIT_TIME);

		//cmd[0] = G370N_RR_SMPL_CTRL_H;	cmd[1] = G370N_CMD_RD;	cmd[2] = G370N_DELIMITER;
		//Sleep(G370N_RSP_WAIT_TIME);

		//// Moving Average Filter, Tap Size = 64
		//cmd[0] = G370N_RR_FILT_CTRL_L;	cmd[1] = G370N_MAF_TS_32;	cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);	Sleep(G370N_RSP_WAIT_TIME);

		//cmd[0] = G370N_RR_FILT_CTRL_L;	cmd[1] = G370N_CMD_RD;	cmd[2] = G370N_DELIMITER;
		//Sleep(G370N_RSP_WAIT_TIME);

		////--------------------------------------------------------------
		//// option 3: Fs = 50, MAF TS = 32
		////--------------------------------------------------------------
		//// Sampling Frequency
		//cmd[0] = G370N_RR_SMPL_CTRL_H;	cmd[1] = G370N_FS_50;	cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);	Sleep(G370N_RSP_WAIT_TIME);

		//cmd[0] = G370N_RR_SMPL_CTRL_H;	cmd[1] = G370N_CMD_RD;	cmd[2] = G370N_DELIMITER;
		//Sleep(G370N_RSP_WAIT_TIME);

		//// Moving Average Filter, Tap Size = 64
		//cmd[0] = G370N_RR_FILT_CTRL_L;	cmd[1] = G370N_MAF_TS_32;	cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);	Sleep(G370N_RSP_WAIT_TIME);

		//cmd[0] = G370N_RR_FILT_CTRL_L;	cmd[1] = G370N_CMD_RD;	cmd[2] = G370N_DELIMITER;
		//Sleep(G370N_RSP_WAIT_TIME);

		//////--------------------------------------------------------------
		////// option 4: Fs = 100, MAF TS = 16
		//////--------------------------------------------------------------
		//// Sampling Frequency
		//TRACE(">> WT : SMPL_CTRL_L - Fs=100\n");
		//cmd[0] = G370N_WR_SMPL_CTRL;
		//cmd[1] = G370N_RV_FS_100;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//TRACE(">> RD : SMPL_CTRL_L\n");
		//cmd[0] = G370N_RR_SMPL_CTRL_L;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//// Moving Average Filter, Tap Size = 16
		//TRACE(">> WT : FILT_CTRL_L - Tap size=16\n");
		//cmd[0] = G370N_WR_FILT_CTRL;
		//cmd[1] = G370N_RV_MAF_TS_16;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		//TRACE(">> RD : FILT_CTRL_L\n");
		//cmd[0] = G370N_RR_FILT_CTRL_L;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);

		////--------------------------------------------------------------
		//// option 4: Fs = 50, MAF TS = 32
		////--------------------------------------------------------------
		// Sampling Frequency
		TRACE(">> WT : SMPL_CTRL_L - Fs=50\n");
		cmd[0] = G370N_WR_SMPL_CTRL;
		if (m_pInfo->dFs_IMU == 100.0)
		{
			cmd[1] = G370N_RV_FS_100;
		}
		else if (m_pInfo->dFs_IMU == 50.0)
		{
			cmd[1] = G370N_RV_FS_50;
		}
		else
		{
			cmd[1] = G370N_RV_FS_100;
		}	
		
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : SMPL_CTRL_L\n");
		cmd[0] = G370N_RR_SMPL_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		// Moving Average Filter, Tap Size = 32
		TRACE(">> WT : FILT_CTRL_L - Tap size=32\n");
		cmd[0] = G370N_WR_FILT_CTRL;

		if (m_pInfo->dTapSize_IMU == 0)
		{
			cmd[1] = G370N_RV_MAF_TS_0;
		}
		else if (m_pInfo->dTapSize_IMU == 2.0)
		{
			cmd[1] = G370N_RV_MAF_TS_2;
		}
		else if (m_pInfo->dTapSize_IMU == 4.0)
		{
			cmd[1] = G370N_RV_MAF_TS_4;
		}
		else if (m_pInfo->dTapSize_IMU == 8.0)
		{
			cmd[1] = G370N_RV_MAF_TS_8;
		}
		else if (m_pInfo->dTapSize_IMU == 16.0)
		{
			cmd[1] = G370N_RV_MAF_TS_16;
		}
		else if (m_pInfo->dTapSize_IMU == 32.0)
		{
			cmd[1] = G370N_RV_MAF_TS_32;
		}
		else if (m_pInfo->dTapSize_IMU == 64.0)
		{
			cmd[1] = G370N_RV_MAF_TS_64;
		}
		else if (m_pInfo->dTapSize_IMU == 128.0)
		{
			cmd[1] = G370N_RV_MAF_TS_128;
		}

		//cmd[1] = G370N_RV_MAF_TS_64;
		//cmd[1] = G370N_RV_MAF_TS_32;
		//cmd[1] = G370N_RV_MAF_TS_16;
		
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : FILT_CTRL_L\n");
		cmd[0] = G370N_RR_FILT_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);


		TRACE("\n\n\n\n\n");
		Sleep(1000);
		TRACE("\n\n\n\n\n");



		////--------------------------------------------------------------
		////	6. UART Auto Mode
		////--------------------------------------------------------------	
		TRACE("6. UART Auto Mode\n");

		TRACE(">> WT : UART_CTRL_L - UART Auto Mode\n");
		cmd[0] = G370N_WR_UART_CTRL_L;
		cmd[1] = G370N_RV_UART_AUTO;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : UART_CTRL_L\n");
		cmd[0] = G370N_RR_UART_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE("\n\n\n");



		////--------------------------------------------------------------
		////	7. UART Baudrate Setting
		////--------------------------------------------------------------
		TRACE("X. UART Baudrate Setting\n");

		TRACE(">> WT : UART_CTRL_H - UART Baudrate Setting\n");
		cmd[0] = G370N_WR_UART_CTRL_H;
		cmd[1] = G370N_RV_UARTBR_460K;
		//cmd[1] = G370N_RV_UARTBR_921K;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		//DCB dcb;
		//dcb.DCBlength = sizeof(DCB);	
		//GetCommState(_serial.idComDev, &dcb);
		//dcb.BaudRate = 921600;
		//SetCommState(_serial.idComDev, &dcb);
		//Sleep(100);

		TRACE(">> RD : UART_CTRL_L\n");
		cmd[0] = G370N_RR_UART_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);


		//printf(">> RD : UART_CTRL_H\n");
		//cmd[0] = G370N_RR_UART_CTRL_H;
		//cmd[1] = G370N_CMD_RD;
		//cmd[2] = G370N_DELIMITER;
		//_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		//Sleep(G370N_RSP_WAIT_TIME);


		TRACE("\n\n\n");




		////--------------------------------------------------------------
		////	X. BURST Mode Setting	// see datasheet
		////--------------------------------------------------------------
		TRACE("X. BURST Mode Setting\n");

		// GPIO=off, COUNT=on, CHKSM=on Setting
		TRACE(">> WT : BRST_CTRL1_L - GPIO=off, COUNT=on, CHKSM=on\n");
		cmd[0] = G370N_WR_BRST_CTRL1_L;
		cmd[1] = 0x03;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : BRST_CTRL1_L\n");
		cmd[0] = G370N_RR_BRST_CTRL1_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		// FLAG=on, TEMP=on, GYRO=on, ACCL=on, DLTA=off, DLTV=off Setting
		TRACE(">> WT : BRST_CTRL1_H - FLAG=on, TEMP=on, GYRO=on, ACCL=on, DLTA=off, DLTV=off\n");
		cmd[0] = G370N_WR_BRST_CTRL1_H;
		cmd[1] = 0xF0;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME * 2);

		TRACE(">> RD : BRST_CTRL1_H\n");
		cmd[0] = G370N_RR_BRST_CTRL1_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME * 2);

		// TEMP=16bit, GYRO=16bit, ACCL=16bit  //DLTA=16bit, DLTV=16bit
		TRACE(">> WT : BRST_CTRL2_H - TEMP=16bit, GYRO=16bit, ACCL=16bit  //DLTA=16bit, DLTV=16bit\n");
		cmd[0] = G370N_WR_BRST_CTRL2_H;
		cmd[1] = 0x00;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : BRST_CTRL2_H\n");
		cmd[0] = G370N_RR_BRST_CTRL2_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE("\n\n\n");




		////--------------------------------------------------------------
		////	X. move to sampling mode
		////--------------------------------------------------------------
		TRACE("X. move to sampling mode\n");

		// WINDOW = 0
		TRACE(">> WT : WIN_CTRL - WINDOW = 0\n");
		cmd[0] = G370N_WR_WIN_CTRL;
		cmd[1] = G370N_RV_WIN_0;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : WIN_CTRL\n");
		cmd[0] = G370N_RR_WIN_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		// move to sampling mode
		TRACE(">> WT : MODE_CTRL_H - Sampling mode\n");
		cmd[0] = G370N_WR_MODE_CTRL;
		cmd[1] = G370N_RV_MOVE_SMPL;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE(">> RD : MODE_CTRL_H\n");
		cmd[0] = G370N_RR_MODE_CTRL_L;
		cmd[1] = G370N_CMD_RD;
		cmd[2] = G370N_DELIMITER;
		_serialIMU.wt_com((LPSTR)cmd, G370N_CMD_LEN);
		Sleep(G370N_RSP_WAIT_TIME);

		TRACE("\n\n\n");
	}

	return 0;
}

int CNTRIP::ConnectGNSS()
{		
	// # NMEA Disable Array
	unsigned char dis_gga[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23 };
	unsigned char dis_gll[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A };
	unsigned char dis_gsa[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31 };
	unsigned char dis_gsv[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38 };
	unsigned char dis_rmc[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F };
	unsigned char dis_vtg[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46 };

	int ret2 = 0;	
	if (ret2 = _serialGPS.OpenPort_GPS(m_pInfo->nPortGNSS, CBR_115200, this))		// F9P
	{
		// # Configuration Data : NMEA Disable
		_serialGPS.wt_com((LPSTR)dis_gga, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_gll, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_gsa, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_gsv, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_rmc, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_vtg, 16);		Sleep(100);

		// # Configuration Data : NMEA Disable
		_serialGPS.wt_com((LPSTR)dis_gga, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_gll, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_gsa, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_gsv, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_rmc, 16);		Sleep(100);
		_serialGPS.wt_com((LPSTR)dis_vtg, 16);		Sleep(100);

		// # Tartget : UART2 / [Baudrate : 115200] Protocol in : RTCM3, Protocol out : RTCM3 
		//unsigned char uart2_rtcm[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x20, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7, 0x08 };
		//_serialGPS.WriteCommBlock((LPSTR)uart2_rtcm, 28);
		//Sleep(10);
		
		unsigned char ena_pvt[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x18, 0xDF };

		for (int k = 0; k < 2; k++)
		{
			_serialGPS.wt_com((LPSTR)ena_pvt, 16);		Sleep(100);
		}
	}

	return 0;
}

int CNTRIP::ConnectNtrip()
{
	UINT uPort = static_cast<UINT>(atoi(m_pInfo->szPORT));
	
	////********** connect server **********////	
	if (m_Sock.Connect( m_pInfo ->szIP, uPort ) == 0){
		DWORD dwErrorCode = GetLastError();
		if ( dwErrorCode == WSAEWOULDBLOCK ) // No Error
		{
			return 1;	
		}
		else
		{
			ReleaseAttribute();
			return -1;
		}
	}

	CString str = "connected to ntrip server/caster ...";
	m_pInfo->pList->AddString(str);
	m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
		
	return 1;
}

int CNTRIP::NavStart()
{
	if (fp_inssol == NULL)
	{
		fp_inssol = fopen("inssol.txt", "w");
	}

	if (fp_inssol2 == NULL)
	{
		fp_inssol2 = fopen("inssol2.txt", "w");
	}

	return 0;
}

int CNTRIP::NavStop()
{
	if (fp_inssol != NULL)
	{
		fclose(fp_inssol);
		fp_inssol = NULL;
	}

	if (fp_inssol2 != NULL)
	{
		fclose(fp_inssol2);
		fp_inssol2 = NULL;
	}

	return 0;
}

LONG CNTRIP::OnReceive(UINT wParam, LONG lParam)
{	
	int i = 0, j = 0, k = 0;		
	char buf[MAXDATASIZE] = {0};
	int recvByte = 0;	
		
	
	if ( m_pInfo->REQ_FLAG & REQ_SRCT )
	{
		if ( GetSourceTable() == -1 )
		{
			ReleaseAttribute();			
			return -1;//---- 스레드에게 메세지전송해서 엔트립이 기능을 비정상종료임을 알려야 한다.<><><><><>
		}
		else
		{
			return  1;//---- 스레드에게 메세지전송해서 엔트립이 기능을 정상적인 종료임을 알려야 한다.<><><><><>
		}
	}
	
		
	recvByte = m_Sock.Receive(buf, MAXDATASIZE-1);
	buf[recvByte] = 0;


	if ( recvByte > 0 )
	{
		//**** Access authentication message "ICY 200 OK" ****//			
		::ZeroMemory(&m_Res, sizeof(NTRIP_Response_struct));
		if ( m_nIsAthenticated == -1 )
		{	
			if ( NTRIPAthenticationProc( buf, recvByte ) == -1 )
			{
				//---- 스레드에게 메세지전송해서 엔트립이 기능을 비정상종료임을 알려야 한다.<><><><><>
				ReleaseAttribute();
				return -1;
			}
			else
			{
				return  1;	// 성공시 여기서 리턴하고, 다음번 메세지부터는 여기를 통과해서
							// 아래 데이터를 받아들이는 루틴으로 들어갈끼다.
			}
		}
				

		////========================================================================		
		////	
		////	bypass RTCM message from NTRIP server to ublox receiver vis serial
		////
		////========================================================================
		_serialGPS.wt_com((LPSTR)buf, recvByte);		
	}// end if ( recvByte > 0 ){

	else if (recvByte == 0)
	{
		// 수동적 종료.... 
		//perror("\n\t recvByte == 0, Server already closed socket");	
		//---- 상대측 서버가 문을 닫았다고 알려준다.
		ReleaseAttribute();
		CString str = "WARNING: disconnected ...server close";
		m_pInfo->pList->AddString(str);
		m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

		return -1;	
	}
	else if (recvByte == SOCKET_ERROR)
	{
		if ( WSAGetLastError() == WSAEWOULDBLOCK )
		{
			//recv()가 진행중이므로 다른 작업후 재시도 한다.
			//printf("\r\n\t WSAEWOULDBLOCK\r\n");
			Sleep(200);				
		}
		else
		{
			//perror(" recv() error : recvByte == SOCKET_ERROR(103)\n" );	
			ReleaseAttribute();
			//---- 비정상 종료임을 알려준다. 소켓 혹은 통신에 문제가 있음.

			CString str = "WARNING: disconnected ...unknown";
			m_pInfo->pList->AddString(str);
			m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

			return -1;	
		}
	}
	else
	{	
		//perror("\n\t Unknown Error \n");	
		return -1;	
	}

	return 1;
}


LONG CNTRIP::OnCloseServer(UINT wParam, LONG lParam)
{	
	ReleaseAttribute();

	CString str = "disconnected ...server close";
	m_pInfo->pList->AddString(str);
	m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	return -1;
}


LONG CNTRIP::OnConnectError(UINT wParam, LONG lParam)
{	
	TRACE("\r\n\t OnConnectError(UINT wParam, LONG lParam) \r\n");
	ReleaseAttribute();
	//---- 접속에실패했음을 알려준다. // 재접속을 시도해야 하는가..
	CString str = "WARNING: connection error ...";
	m_pInfo->pList->AddString(str);
	m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	return -1;
}


LONG CNTRIP::OnConnectNoError(UINT wParam, LONG lParam)
{	
	////********** SEND message **********////
	char reqMsg[MAXDATASIZE] = {0};	// MAXDATASIZE 1536	// MAX_PATH = 256 byte
	int	sendByte = 0;
	if (GetRequestMsg( reqMsg, /*MAXDATASIZE*/_countof(reqMsg),  sendByte ) == -1)
	{		
		ReleaseAttribute();	
		return -1;
	}
	
	//while (mf_latd == 0 || mf_lond == 0)
	//{
	//	Sleep(10);
	//}

	int cnt = m_Sock.Send(reqMsg, sendByte);

	if ( cnt == sendByte )
	{		
		CString str = "request message is sent...";
		m_pInfo->pList->AddString(str);
		m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
	}
	else if ( cnt == SOCKET_ERROR )
	{
		DWORD dwErrorCode = GetLastError();
		if ( dwErrorCode == WSAEWOULDBLOCK )
		{
			// 정상이다.
		}
		else
		{
			ReleaseAttribute();
			//---- 요청 메세지 전송에 실패했음을 알려준다. 아니면 재전송 시도할까?
			CString str = "WARNING: send request message error...";
			m_pInfo->pList->AddString(str);
			m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
			return -1;
		}		
	}

	return 1;
}

void CNTRIP::OnDestroy()
{
	CFrameWnd::OnDestroy();
	//PostQuitMessage(0);
}

void CNTRIP::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == 1 && m_pInfo != NULL){
		if (m_pInfo->pList->GetCount() > 1000)
			m_pInfo->pList->ResetContent();
	}

	CFrameWnd::OnTimer(nIDEvent);
}




////============================================================================
////	
////	2020.12.21 - SerialPort 추가
////	2021.03.17 - 전반적인 코드 수정 (GPS, IMU, PORT, header)
////	
////
////============================================================================
#define MAXBLOCK        512

DWORD BAUD_RATE_TABLE[] = { CBR_110,CBR_300,CBR_600,CBR_1200,CBR_2400,CBR_4800,CBR_9600,CBR_14400,
							CBR_19200,CBR_38400,CBR_56000,CBR_57600,CBR_115200,230400,460800,921600,0 };



// Event Watch용 쓰레드 함수
DWORD CommWatchProc_GPS(LPSTR lpData);	// for GPS
DWORD CommWatchProc_IMU(LPSTR lpData);	// for IMU

BOOL Port::OpenPort_GPS(long port_number, long baud_rate_select, void* pParent)
{

	// 포트 상태 관련 변수들을 초기화 한다

	idComDev = 0;
	fConnected = FALSE;
	fLocalEcho = FALSE;
	fAutoWrap = TRUE;
	//dwBaudRate				= BAUD_RATE_TABLE[baud_rate_select];
	dwBaudRate = baud_rate_select;
	bByteSize = 8;
	bFlowCtrl = FC_XONXOFF;
	bParity = NOPARITY;
	bStopBits = ONESTOPBIT;
	fXonXoff = FALSE;
	fUseCNReceive = TRUE;
	fDisplayErrors = TRUE;
	osWrite.Offset = 0;
	osWrite.OffsetHigh = 0;
	osRead.Offset = 0;
	osRead.OffsetHigh = 0;


	// Overlapped I/O에 쓰이는 Event 객체들을 생성

	// Read를 위한 Event 객체 생성
	osRead.hEvent = CreateEvent(
		NULL,
		TRUE,
		FALSE,
		NULL);
	if (osRead.hEvent == NULL)
	{
		return FALSE;
	}

	// Write를 위한 Event 객체 생성
	osWrite.hEvent = CreateEvent(
		NULL,
		TRUE,
		FALSE,
		NULL);

	if (osWrite.hEvent == NULL)
	{
		CloseHandle(osRead.hEvent);
		return FALSE;
	}


	// 포트를 생성한다
	char temp[20];
	strcpy(temp, "\\\\.\\");
	sprintf(port_name, "COM%d", port_number);
	//sprintf(port_name, "COM%d", 4);
	strcat(temp, port_name);
	//idComDev = CreateFile((LPCSTR)temp,
	//	GENERIC_READ | GENERIC_WRITE,
	//	0,							
	//	NULL,						
	//	OPEN_EXISTING,
	//	FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,	// Overlapped I/O
	//	NULL);
	//idComDev = CreateFile(L"\\\\.\\COM6", //C099
	idComDev = CreateFile((LPCSTR)temp,	// F9P
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,	// Overlapped I/O
		NULL);

	if (idComDev == INVALID_HANDLE_VALUE)
	{
		CloseHandle(osRead.hEvent);
		CloseHandle(osWrite.hEvent);
		return FALSE;
	}

	SetCommMask(idComDev, EV_RXCHAR);
	SetupComm(idComDev, 4096, 4096);	// 버퍼 설정
	PurgeComm(idComDev, PURGE_TXABORT | PURGE_RXABORT |
		PURGE_TXCLEAR | PURGE_RXCLEAR);	// 버퍼의 모든 데이타를 지운다


	// Overlapped I/O를 위한 Time Out 설정
	COMMTIMEOUTS  CommTimeOuts;

	CommTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 1000;

	//CommTimeOuts.WriteTotalTimeoutMultiplier = 2*CBR_9600/dwBaudRate ; // CBR_9600 기준 ms당 바이트를 두배까지 설정
	CommTimeOuts.WriteTotalTimeoutMultiplier = 2 * CBR_115200 / dwBaudRate; // CBR_9600 기준 ms당 바이트를 두배까지 설정
	//CommTimeOuts.WriteTotalTimeoutMultiplier = 2 * 460800 / dwBaudRate; // CBR_9600 기준 ms당 바이트를 두배까지 설정



	CommTimeOuts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(idComDev, &CommTimeOuts);

	// 포트를 사용가능 상태로 만들고 Event를 감시할 쓰레드를 생성한다
	if (SetupConnection() == TRUE)
	{
		fConnected = TRUE;

		// 쓰레드 생성
		hWatchThread = CreateThread(
			(LPSECURITY_ATTRIBUTES)NULL,
			0,
			(LPTHREAD_START_ROUTINE)CommWatchProc_GPS,
			(LPVOID)pParent,
			0,
			&dwThreadID);

		if (hWatchThread == NULL)	// 쓰레드 생성 실패
		{
			fConnected = FALSE;

			CloseHandle(osRead.hEvent);
			CloseHandle(osWrite.hEvent);
			CloseHandle(idComDev);
			return FALSE;
		}
		else
		{
			// 장치에 DTR(Data-Terminal-Ready)을 알린다
			EscapeCommFunction(idComDev, SETDTR);
		}
	}
	else
	{
		fConnected = FALSE;

		CloseHandle(osRead.hEvent);
		CloseHandle(osWrite.hEvent);
		CloseHandle(idComDev);
		return FALSE;
	}
	return TRUE;
}

BOOL Port::OpenPort_IMU(long port_number, long baud_rate_select, void* pParent)
{

	// 포트 상태 관련 변수들을 초기화 한다

	//idComDev = 0;
	//fConnected = FALSE;
	//fLocalEcho = FALSE;
	//fAutoWrap = TRUE;
	////dwBaudRate				= BAUD_RATE_TABLE[baud_rate_select];
	//dwBaudRate = baud_rate_select;
	//bByteSize = 8;
	//bFlowCtrl = FC_XONXOFF;
	//bParity = NOPARITY;
	//bStopBits = ONESTOPBIT;
	//fXonXoff = FALSE;
	//fUseCNReceive = TRUE;
	//fDisplayErrors = TRUE;
	//osWrite.Offset = 0;
	//osWrite.OffsetHigh = 0;
	//osRead.Offset = 0;
	//osRead.OffsetHigh = 0;


	idComDev = 0;
	fConnected = FALSE;
	fLocalEcho = FALSE;
	fAutoWrap = TRUE;
	dwBaudRate = baud_rate_select;
	bByteSize = 8;
	bFlowCtrl = 0;
	bParity = NOPARITY;	////
	bStopBits = ONESTOPBIT;	////
	fXonXoff = FALSE;
	fUseCNReceive = FALSE;
	fDisplayErrors = FALSE;
	osWrite.Offset = 0;
	osWrite.OffsetHigh = 0;
	osRead.Offset = 0;
	osRead.OffsetHigh = 0;


	// Overlapped I/O에 쓰이는 Event 객체들을 생성

	// Read를 위한 Event 객체 생성
	osRead.hEvent = CreateEvent(
		NULL,
		TRUE,
		FALSE,
		NULL);
	if (osRead.hEvent == NULL)
	{
		return FALSE;
	}

	// Write를 위한 Event 객체 생성
	osWrite.hEvent = CreateEvent(
		NULL,
		TRUE,
		FALSE,
		NULL);

	if (osWrite.hEvent == NULL)
	{
		CloseHandle(osRead.hEvent);
		return FALSE;
	}


	// 포트를 생성한다
	char temp[20];
	strcpy(temp, "\\\\.\\");
	sprintf(port_name, "COM%d", port_number);
	strcat(temp, port_name);
	//idComDev = CreateFile((LPCSTR)temp,
	//	GENERIC_READ | GENERIC_WRITE,
	//	0,							
	//	NULL,						
	//	OPEN_EXISTING,
	//	FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,	// Overlapped I/O
	//	NULL);
	//idComDev = CreateFile(L"\\\\.\\COM6", //C099
	idComDev = CreateFile((LPCSTR)temp,	// F9P
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,	// Overlapped I/O
		NULL);

	if (idComDev == INVALID_HANDLE_VALUE)
	{
		CloseHandle(osRead.hEvent);
		CloseHandle(osWrite.hEvent);
		return FALSE;
	}

	SetCommMask(idComDev, EV_RXCHAR);
	SetupComm(idComDev, 4096, 4096);	// 버퍼 설정
	PurgeComm(idComDev, PURGE_TXABORT | PURGE_RXABORT |
		PURGE_TXCLEAR | PURGE_RXCLEAR);	// 버퍼의 모든 데이타를 지운다


	// Overlapped I/O를 위한 Time Out 설정
	COMMTIMEOUTS  CommTimeOuts;

	CommTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 1000;

	//CommTimeOuts.WriteTotalTimeoutMultiplier = 2*CBR_9600/dwBaudRate ; // CBR_9600 기준 ms당 바이트를 두배까지 설정
	CommTimeOuts.WriteTotalTimeoutMultiplier = 2 * CBR_9600 / dwBaudRate; // CBR_9600 기준 ms당 바이트를 두배까지 설정
	//CommTimeOuts.WriteTotalTimeoutMultiplier = 2 * 460800 / dwBaudRate; // CBR_9600 기준 ms당 바이트를 두배까지 설정



	CommTimeOuts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(idComDev, &CommTimeOuts);

	// 포트를 사용가능 상태로 만들고 Event를 감시할 쓰레드를 생성한다
	if (SetupConnection() == TRUE)
	{
		fConnected = TRUE;

		// 쓰레드 생성
		hWatchThread = CreateThread(
			(LPSECURITY_ATTRIBUTES)NULL,
			0,
			(LPTHREAD_START_ROUTINE)CommWatchProc_IMU,
			(LPVOID)pParent,
			0,
			&dwThreadID);

		if (hWatchThread == NULL)	// 쓰레드 생성 실패
		{
			fConnected = FALSE;

			CloseHandle(osRead.hEvent);
			CloseHandle(osWrite.hEvent);
			CloseHandle(idComDev);
			return FALSE;
		}
		else
		{
			// 장치에 DTR(Data-Terminal-Ready)을 알린다
			EscapeCommFunction(idComDev, SETDTR);
		}
	}
	else
	{
		fConnected = FALSE;

		CloseHandle(osRead.hEvent);
		CloseHandle(osWrite.hEvent);
		CloseHandle(idComDev);
		return FALSE;
	}
	return TRUE;
}

BOOL Port::SetupConnection(void)
{
	// DCB 구조체를 이용하여 포트를 셋팅한다
	BYTE       bSet;
	DCB        dcb;

	dcb.DCBlength = sizeof(DCB);

	GetCommState(idComDev, &dcb);

	dcb.BaudRate = dwBaudRate;
	dcb.ByteSize = bByteSize;
	dcb.Parity = bParity;
	dcb.StopBits = bStopBits;

	// setup hardware flow control

	bSet = (BYTE)((bFlowCtrl & FC_DTRDSR) != 0);
	dcb.fOutxDsrFlow = bSet;
	if (bSet)
		dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
	else
		dcb.fDtrControl = DTR_CONTROL_ENABLE;

	bSet = (BYTE)((bFlowCtrl & FC_RTSCTS) != 0);
	dcb.fOutxCtsFlow = bSet;
	if (bSet)
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
	else
		dcb.fRtsControl = RTS_CONTROL_ENABLE;

	// setup software flow control

	bSet = (BYTE)((bFlowCtrl & FC_XONXOFF) != 0);

	dcb.fInX = dcb.fOutX = bSet;
	dcb.XonChar = ASCII_XON;
	dcb.XoffChar = ASCII_XOFF;
	dcb.XonLim = 100;
	dcb.XoffLim = 100;

	// other various settings

	dcb.fBinary = TRUE;
	dcb.fParity = TRUE;

	return SetCommState(idComDev, &dcb);
}

long Port::rd_com(LPSTR lpszBlock, long nMaxLength)
{
	BOOL       fReadStat;
	COMSTAT    ComStat;
	DWORD      dwErrorFlags;
	DWORD      dwLength;
	DWORD      dwError;
	char       szError[10];

	// 큐에서 읽어야 할 데이타 크기를 가져온다
	ClearCommError(idComDev, &dwErrorFlags, &ComStat);
	//dwLength = min( (DWORD) nMaxLength, ComStat.cbInQue ) ;
	dwLength = ((DWORD)nMaxLength >= ComStat.cbInQue) ? ComStat.cbInQue : (DWORD)nMaxLength;


	if (dwLength > 0)	// 읽어야 할 데이타가 있는 경우
	{
		// 데이타를 읽는다. Overlapped I/O임을 주의.
		fReadStat = ReadFile(idComDev, lpszBlock,
			dwLength, &dwLength, &osRead);
		if (!fReadStat)	// 읽어야 할 바이트를 다 읽지 못했다
		{
			if (GetLastError() == ERROR_IO_PENDING)	// I/O Pending에 의해 다 읽지 못한 경우
			{
				// I/O가 완료되기를 기다린다.
				while (!GetOverlappedResult(idComDev,
					&osRead, &dwLength, TRUE))
				{
					dwError = GetLastError();
					if (dwError == ERROR_IO_INCOMPLETE)	// I/O가 아직 끝나지 않았다
						continue;
					else	// 에러 발생
					{
						sprintf(szError, "<CE-%u>\n\r", dwError);
						printf(szError);
						// 에러를 클리어 하고 다른 I/O가 가능하도록 한다
						ClearCommError(idComDev, &dwErrorFlags, &ComStat);
						break;
					}

				}

			}
			else // I/O Pending이 아닌 다른 에러가 발생한 경우
			{
				dwLength = 0;
				// 에러를 클리어 하고 다른 I/O가 가능하도록 한다
				ClearCommError(idComDev, &dwErrorFlags, &ComStat);
			}
		}
	}

	return (dwLength);

}

BOOL Port::wt_com(LPSTR lpByte, DWORD dwBytesToWrite)
{

	BOOL        fWriteStat;
	DWORD       dwBytesWritten;
	DWORD       dwErrorFlags;
	DWORD   	dwError;
	DWORD       dwBytesSent = 0;
	COMSTAT     ComStat;
	char        szError[128];

	fWriteStat = WriteFile(idComDev, lpByte, dwBytesToWrite,
		&dwBytesWritten, &osWrite);

	if (!fWriteStat)	// 써야할 바이트를 다 쓰지 못했다
	{
		if (GetLastError() == ERROR_IO_PENDING)	// I/O Pending에 의한 경우
		{
			// I/O가 완료되기를 기다린다
			while (!GetOverlappedResult(idComDev,
				&osWrite, &dwBytesWritten, TRUE))
			{
				dwError = GetLastError();
				if (dwError == ERROR_IO_INCOMPLETE)
				{
					// 보낸 전체 바이트 수를 체크
					dwBytesSent += dwBytesWritten;
					continue;
				}
				else
				{
					// 에러 발생
					sprintf(szError, "<CE-%u>", dwError);
					printf("%s\r\n", szError);
					//WriteTTYBlock( hWnd, szError, lstrlen( szError ) ) ;
					ClearCommError(idComDev, &dwErrorFlags, &ComStat);
					break;
				}
			}

			dwBytesSent += dwBytesWritten;

			if (dwBytesSent != dwBytesToWrite)	// 보내야 할 바이트와 보낸 바이트가 일치하지 않는 경우
			{
				sprintf(szError, "\nProbable Write Timeout: Total of %ld bytes sent", dwBytesSent);
				OutputDebugString((LPCSTR)szError);
			}				
			//else	// 성공적으로 모두 보낸 경우
			//	sprintf(szError, "\n%ld bytes written", dwBytesSent);

			//OutputDebugString((LPCSTR)szError);
		}
		else // I/O Pending 외의 다른 에러
		{
			ClearCommError(idComDev, &dwErrorFlags, &ComStat);
			return FALSE;
		}
	}


	return TRUE;

}

BOOL Port::ClosePort(void)
{
	fConnected = FALSE;

	// 이벤트 발생을 중지한다
	SetCommMask(idComDev, 0);

	// Event Watch 쓰레드가 중지되기를 기다린다
	while (dwThreadID != 0);


	// DTR(Data-Terminal-Ready) 시그널을 Clear 한다
	EscapeCommFunction(idComDev, CLRDTR);

	// 대기중인 모든 데이타를 지운다	
	PurgeComm(idComDev, PURGE_TXABORT | PURGE_RXABORT |
		PURGE_TXCLEAR | PURGE_RXCLEAR);

	// 핸들을 반환한다
	CloseHandle(osRead.hEvent);
	CloseHandle(osWrite.hEvent);
	CloseHandle(idComDev);

	return TRUE;

}



////============================================================================
////
////	Event Watch 쓰레드	// UBX PVT case
////
////============================================================================
DWORD CommWatchProc_GPS(LPSTR lpData)
{
	DWORD       dwEvtMask;

	//Port		*pp = (Port *)lpData;	
	CNTRIP		*pNtrip = (CNTRIP *)lpData;
	Port		*pPort = (Port *)(&pNtrip->_serialGPS);
	//FILE		*fp = pPort->m_fpGPS;
	OVERLAPPED  os;
	long        nLength;
	BYTE       abIn[MAXBLOCK + 1];

	memset(&os, 0, sizeof(OVERLAPPED));


	pNtrip->pre_gpst = 0;
	pNtrip->cur_gpst = 0;
	char str_gpst[6] = { 0 };
	pNtrip->isgpsupdated = 0;

	// Event 객체 생성
	os.hEvent = CreateEvent(
		NULL,
		TRUE,
		FALSE,
		NULL);

	if (os.hEvent == NULL)
		return FALSE;

	if (!SetCommMask(pPort->idComDev, EV_RXCHAR))
	{
		CloseHandle(os.hEvent);
		return FALSE;
	}

	int isfilesave = 0;


	__int64 _tstart1 = 0;
	__int64 _tend1 = 0;
	__int64 _telapsed1 = 0;
	__int64 _freq1 = 0;
	char sztime[10] = { 0 };


	PVTMSG pvt;
	PVTMSG2 pvt2;

	
	char szlog[1024] = { 0 };

	int cursor = 0;
	//int is_GPS_available = 0;


	////--------------------------------------------------------------
	////
	////--------------------------------------------------------------
	while (pPort->fConnected)
	{
		dwEvtMask = 0;
		WaitCommEvent(pPort->idComDev, &dwEvtMask, NULL);
		if ((dwEvtMask & EV_RXCHAR) == EV_RXCHAR)
		{
			do
			{
				if (nLength = pPort->rd_com((LPSTR)abIn, MAXBLOCK))
				{
					// 1) copy
					memset((void*)pPort->rxbuff, 0, MAX_SERIAL_LEN);
					memcpy((void*)pPort->rxbuff, (void*)abIn, nLength - 1);
					pPort->rxbuff[nLength - 2] = '\n';

					// 2) check UBX-NAV-PVT Message Header 2 bytes, Class 1 byte, and ID 1 byte
					if (pPort->rxbuff[0] != 0xb5 || pPort->rxbuff[1] != 0x62 || pPort->rxbuff[2] != 0x01 || pPort->rxbuff[3] != 0x07)
						continue;

					// 3) check UBX-NAV-PVT Message length (exclude header, class, id, checksum..)
					if (nLength < 92)	// length of "$GPGGA,HHMMSS" = 13
						continue;
					
					// 4) Parsing
					memset((void*)&pvt, 0, sizeof(PVTMSG));
					memset((void*)&pvt2, 0, sizeof(PVTMSG2));
					cursor = 6;	// skip message header
					
					memcpy(&pvt.iTOW, pPort->rxbuff + cursor, sizeof(unsigned int)); cursor += sizeof(unsigned int);
					memcpy(&pvt.year, pPort->rxbuff + cursor, sizeof(unsigned short)); cursor += sizeof(unsigned short);
					memcpy(&pvt.month, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					memcpy(&pvt.day, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					memcpy(&pvt.hour, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					memcpy(&pvt.minute, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					memcpy(&pvt.second, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					cursor = cursor + 9;
					memcpy(&pvt.fixType, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					memcpy(&pvt.flags, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					cursor = cursor + 1;
					memcpy(&pvt.numSV, pPort->rxbuff + cursor, sizeof(unsigned char)); cursor += sizeof(unsigned char);
					memcpy(&pvt.lon, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.lat, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.height, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.hMSL, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.hAcc, pPort->rxbuff + cursor, sizeof(unsigned int)); cursor += sizeof(unsigned int);
					memcpy(&pvt.vAcc, pPort->rxbuff + cursor, sizeof(unsigned int)); cursor += sizeof(unsigned int);
					memcpy(&pvt.velN, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.velE, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.velD, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.gSpeed, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					memcpy(&pvt.headMot, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);
					cursor = cursor + 8;
					memcpy(&pvt.pDOP, pPort->rxbuff + cursor, sizeof(unsigned short)); cursor += sizeof(unsigned short);
					cursor = cursor + 6;
					memcpy(&pvt.headingVehicle, pPort->rxbuff + cursor, sizeof(int)); cursor += sizeof(int);


					// 5) conversion (unit)
					pvt2.iTOW = (double)(pvt.iTOW * 0.001);	// second
					pvt2.year = (int)(pvt.year);
					pvt2.month = (int)(pvt.month);
					pvt2.day = (int)(pvt.day);
					pvt2.hour = (int)(pvt.hour);
					pvt2.minute = (int)(pvt.minute);
					pvt2.second = (double)(pvt.second);
					pvt2.fixType = pvt.fixType;
					pvt2.flags = pvt.flags;
					pvt2.numSV = pvt.numSV;
					pvt2.lon = (double)(pvt.lon * 1e-7);	// deg
					pvt2.lat = (double)(pvt.lat * 1e-7);	// deg
					pvt2.height = (double)(pvt.height * 0.001);	// meter
					pvt2.hMSL = (double)(pvt.hMSL * 0.001);	// meter
					pvt2.hAcc = (double)(pvt.hAcc * 0.001);	// meter
					pvt2.vAcc = (double)(pvt.vAcc * 0.001);	// meter
					pvt2.velN = (double)(pvt.velN * 0.001);	// m/s
					pvt2.velE = (double)(pvt.velE * 0.001);	// m/s
					pvt2.velD = (double)(pvt.velD * 0.001);	// m/s
					pvt2.gSpeed = (double)(pvt.gSpeed * 0.001);	// m/s
					pvt2.headMot = (double)(pvt.headMot * 1e-5);	// deg
					pvt2.headingVehicle = (double)(pvt.headingVehicle * 1e-5);	// deg
					pvt2.pDOP = (double)(pvt.pDOP * 0.01);	//


					// 6) open GPS log file
					if (pPort->m_fpGPS == NULL)
					{
						pNtrip->mf_iTOW = (double)(pvt.iTOW * 0.001);	// 01	// second
						pNtrip->mf_iTOW = (int)(pvt.year);				// 02
						pNtrip->mf_iTOW = (int)(pvt.month);				// 03
						pNtrip->mf_iTOW = (int)(pvt.day);				// 04
						pNtrip->mf_iTOW = (int)(pvt.hour);				// 05
						pNtrip->mf_iTOW = (int)(pvt.minute);			// 06
						pNtrip->mf_iTOW = (double)(pvt.second);			// 07

						pNtrip->mf_latd = (double)(pvt.lon * 1e-7);		// 08	// deg
						pNtrip->mf_lond = (double)(pvt.lat * 1e-7);		// 09	// deg
						pNtrip->mf_hgtm = (double)(pvt.height * 0.001);	// 10	// meter

						pPort->m_fpGPS = fopen("log_GPS.txt", "wt");
						CString str = "OPEN GNSS MEASUREMENT FILE";
						pNtrip->m_pInfo->pList->AddString(str);
						pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
					}
								
					// 7)
					////--------------------------------------------------------
					////	GPS OFF 상황시
					////	 - 1. GPS Log file 기록 유지
					////	 - 2. 알고리즘은 bypass (GPS 측정치 업데이트 안됨)
					////
					////	 - added at 2021.07.16
					////--------------------------------------------------------
					if (pvt2.fixType == 0 || pvt2.flags == 0 || pvt2.pDOP == 99.0)
					{
						// TODO
						CString str = "No GPS signals ......";
						pNtrip->m_pInfo->pList->AddString(str);
						pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
						//continue;
						pNtrip->m_is_GPS_available = 0;
					}
					else
					{
						pNtrip->m_is_GPS_available = 1;
					}

					// 7-1) USER COMMAND for Unavailable GPS Situation
					if (pNtrip->m_pInfo->engage_gnss == 0)
						pNtrip->m_is_GPS_available = 0;


					// 8) update gps time (iTOW)
					pNtrip->pre_gpst = pNtrip->cur_gpst;
					pNtrip->cur_gpst = pvt.iTOW;
					//pNtrip->cur_gpst = pvt2.iTOW;
						
					
					// 11) save to struct gps_sol
					pNtrip->gps_sol.gps_time = pvt2.iTOW;	// sec
					pNtrip->gps_sol.lat = pvt2.lat;			// deg
					pNtrip->gps_sol.lon = pvt2.lon;			// deg
					pNtrip->gps_sol.hgt = pvt2.height;		// meter
					pNtrip->gps_sol.vel[0] = pvt2.velN;		// m/s
					pNtrip->gps_sol.vel[1] = pvt2.velE;		// m/s
					pNtrip->gps_sol.vel[2] = pvt2.velD;		// m/s
					pNtrip->gps_sol.vel_horz = pvt2.gSpeed;	// m/s
					pNtrip->gps_sol.hd = pvt2.headMot;		// deg


					// 9) check GPS time for IMU sync.
					//if (pNtrip->m_is_GPS_available)
					//{						
						if ((pNtrip->pre_gpst != 0 && pNtrip->cur_gpst != 0) && (pNtrip->cur_gpst > pNtrip->pre_gpst))
						{
							pNtrip->isgpsupdated = 1;
							////------------------------------------------------
							////	GNSS ON/OFF
							////------------------------------------------------
							if (pNtrip->m_pInfo->engage_nav)
							{
								if ((pNtrip->imu_sts & IMU_STS_GNSS_READY) == 0)
								{
									pNtrip->imu_sts |= IMU_STS_GNSS_READY;
									CString str = "IMU_STS_GNSS_READY";
									pNtrip->m_pInfo->pList->AddString(str);
									pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
								}
							} // if (pNtrip->m_pInfo->engage_nav)					
						}
					//}
					

					//// Check Sampling Period of GNSS // debug
					if (0)
					{
						if (_tstart1 == 0)
						{
							TIME_START(_tstart1, _freq1);
						}
						TIME_END(_tstart1, _tend1, _telapsed1, _freq1);
					}
					
					// 10) record measurement					
					if (pPort->m_fpGPS != NULL)
					{						
						memset((void*)szlog, 0, 1024);
						//sprintf(szlog, "%12.2f, %04d,%02d,%02d, %02d,%02d,%5.2f, %3d,%3d, %12.7f, %13.7f, %9.3f, %9.3f, %.3f,%.3f, %.3f,%.3f,%.3f,%.3f, %.5f,%.5f, %.2f\n",
						sprintf(szlog, "%12.2f\t%04d\t%02d\t%02d\t%02d\t%02d\t%5.2f\t%3d\t%3d\t%12.7f\t%13.7f\t%9.3f\t%9.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.5f\t%.5f\t%.2f\n",
							pvt2.iTOW,																	// 01
							pvt2.year, pvt2.month, pvt2.day, pvt2.hour, pvt2.minute, pvt2.second,		// 02~07
							(int)pvt2.fixType, (int)pvt2.flags,											// 08~09
							pvt2.lat, pvt2.lon, pvt2.height, pvt2.hMSL,									// 10~13
							pvt2.hAcc, pvt2.vAcc,														// 14~15
							pvt2.velN, pvt2.velE, pvt2.velD, pvt2.gSpeed,								// 16~19
							pvt2.headMot, pvt2.headingVehicle, pvt2.pDOP);								// 20~22
						fprintf(pPort->m_fpGPS, "%s", szlog);
												
						//// update current llh for vrs request message
						pNtrip->mf_latd = pvt2.lat;		// deg
						pNtrip->mf_lond = pvt2.lon;		// deg
						pNtrip->mf_hgtm = pvt2.height;	// meter
						
						//// update list box
						//sprintf(szlog, "%12.2f, %3d,%3d, %12.7f, %13.7f, %9.3f, %.3f, %.5f, %.2f\n",
						//	pvt2.iTOW,							
						//	(int)pvt2.fixType, (int)pvt2.flags,
						//	pvt2.lat, pvt2.lon, pvt2.height,							
						//	pvt2.gSpeed, pvt2.headMot, pvt2.pDOP);						
						//pNtrip->m_pInfo->pList->AddString((LPCTSTR)(szlog));
						//pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
					}
					

					// 12) GPS/IMU Algorithm
					if (pNtrip->m_is_GPS_available && pNtrip->m_pInfo->engage_nav)
					{	
						//// check & change status
						if ((pNtrip->imu_sts & (IMU_STS_IMU_READY | IMU_STS_GNSS_READY)) == (IMU_STS_IMU_READY | IMU_STS_GNSS_READY))
						{
							if (!(pNtrip->imu_sts & IMU_STS_COARSE_ALIGN))
							{
								pNtrip->imu_sts |= IMU_STS_COARSE_ALIGN;
								CString str = "IMU_STS_COARSE_ALIGN - GNSS Receiver";
								pNtrip->m_pInfo->pList->AddString(str);
								pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
							}
						}
						
						//// initialize & coarse alignment
						if ((pNtrip->imu_sts & IMU_STS_COARSE_ALIGN) && !(pNtrip->imu_sts & IMU_STS_EKF))
						{
							//if (pNtrip->ini_imu_flag && pNtrip->gps_sol.vel_horz > 1.0)
							if (pNtrip->ini_imu_flag && pNtrip->gps_sol.vel_horz > 0.2)
							//if (pNtrip->ini_imu_flag && pNtrip->gps_sol.vel_horz > 0.00001)
							{
								nav_type_init_tight(&(pNtrip->avr), &(pNtrip->h), &(pNtrip->m), &(pNtrip->pps));
								course_align(&(pNtrip->avr), &(pNtrip->h), &(pNtrip->m), &(pNtrip->gps_sol));

								init_nav_by_lgps(pNtrip->gps_sol, pNtrip->m, pNtrip->h);
								tframe_covar_tight(&(pNtrip->pps));

								pNtrip->ini_gpstime = pNtrip->gps_sol.gps_time;
								pNtrip->gps_sol.delay = pNtrip->pps.delay;
								pNtrip->ini_imu_flag = 0;
								pNtrip->pps.gps_time_last_propa = pNtrip->ini_gpstime;

								pNtrip->imu_sts |= IMU_STS_EKF;

								CString str;
								str.Format("Coarse Align >> RPY(deg) = %.3f,  %.3f,  %.3f", pNtrip->avr.ca_roll, pNtrip->avr.ca_pitch, pNtrip->avr.ca_yaw);
								pNtrip->m_pInfo->pList->AddString(str);
								pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
							}
						}
					} // end if (pNtrip->m_pInfo->engage_nav)				
				}
			} while (nLength > 0);
		}
		else
		{
			TRACE("<Other Event>\r\n");
		}
	}
	
	CloseHandle(os.hEvent);

	pPort->dwThreadID = 0;
	pPort->hWatchThread = NULL;

	CString str = "GNSS Loop End & Port is Closed...";
	pNtrip->m_pInfo->pList->AddString(str);
	pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	if (pPort->m_fpGPS != NULL)
	{
		fclose(pPort->m_fpGPS);
		pPort->m_fpGPS = NULL;
		CString str = "CLOSE GNSS MEASUREMENT FILE";
		pNtrip->m_pInfo->pList->AddString(str);
		pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
	}

	return TRUE;
}


////============================================================================
////
////	Event Watch 쓰레드	// IMU Burst Data Packet
////
////============================================================================
DWORD CommWatchProc_IMU(LPSTR lpData)
{
	CNTRIP		*pNtrip = (CNTRIP *)lpData;
	Port		*pPort = (Port *)(&(pNtrip->_serialIMU));
	double		fs = pNtrip->m_pInfo->dFs_IMU;

	OVERLAPPED  os;
	DWORD       dwEvtMask;	
	

	char szlog[1024] = { 0 };

	memset(&os, 0, sizeof(OVERLAPPED));
	os.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (os.hEvent == NULL)
		return FALSE;

	if (!SetCommMask(pPort->idComDev, EV_RXCHAR))
	{
		CloseHandle(os.hEvent);
		return FALSE;
	}

	//TRACE("\n\t - IMU CommWatchProc Enabled ...\n\n");



	// debug, to check a sampling period
	//	see QueryPerformanceCounter(..) (Windows API) & QueryPerformanceFrequency(..)
	__int64 _tstart = 0;	// time at start of measure
	__int64 _tend = 0;		// time at end of measure 
	__int64 _telapsed = 0;	// elapsed time
	__int64 _freq = 0;		// clock frequency	




	// respond message
	BYTE WND = 0;	// register window value. G370N has two window for the registers, 0 or 1.
	BYTE ADDR = 0;	// address byte
	BYTE MSB = 0;	// most significant byte
	BYTE LSB = 0;	// least significant byte
	BYTE DLMT = 0;	// delimiter byte



	// respond message : sampling rate, filter type and filter tap size
	enum { eMAF = 0, eKAISER = 1 };
	double smpl_rate = 0;
	int filter_type = 0;
	int tap_size = 0;




	// respond message : burst data packet length / output data type / output data format(# bits)
	//	any combination of each contents is available but sequence of each contents in the burst data packet is determined.
	//	see datasheet... and order of enumeration below
	enum { eFLAG = 0, eTEMP = 1, eGYRO = 2, eACCL = 3, eDLTA = 4, eDLTV = 5, eGPIO = 6, eSCNT = 7, eCKSM = 8, eNTYPES = 9 };
	int ar2[eNTYPES][2] = { 0 };// [][0]: use flag, [][1]: # bytes	
	int BURST_PKT_LEN = 0;		// = sum (ar2[i][0]*ar2[i][1]) + 1 (address byte) + 1 (delimiter byte)




	// burst data packet validity flag
	BYTE ND = 0;	// Flag, New Data Byte
	BYTE EA = 0;	// Flag, All Error Byte
	BYTE fRO = 0;	// Flag, Range Over in ND	// 1: range over occurs
	BYTE fRTD = 0;	// Flag, Run Time Diagnosis in ND	// 1: more than one abnormality occurs
	BYTE fEA = 0;	// Flag, // 1 for at least one failure is found
	unsigned int chksm = 0;	// calculated check sum
	//BYTE fBegin = 0;
	//BYTE fEnd = 0;



	// burst data packet
	G370_BRST rxpkt;





	// flow control
	// con : concatenated,  rem : remained
	long rxd_len = 0;
	BYTE rxdata[G370N_MAXBLOCK] = { 0 };
	BYTE rxdata_rem[G370N_MAXBLOCK] = { 0 };
	BYTE rxdata_con[G370N_MAXBLOCK2] = { 0 };
	int rxd_len_rem = 0;
	int rxd_len_con = 0;
	int sidx = 0;
	int eidx = 0;




	//
	//
	BYTE rx_rsp[G370N_RSP_LEN] = { 0 };			// respond message
	BYTE rx_brst[G370N_MAXBRST_LEN] = { 0 };	// burst data packet
	//
	//

	



	pPort->fstsimu = 0;
	int cnt_rec = 0;
	unsigned int ncounter = 0;
	int sync_cnt = 0;
	while (pPort->fConnected)
	{
		dwEvtMask = 0;
		WaitCommEvent(pPort->idComDev, &dwEvtMask, NULL);

		if ((dwEvtMask & EV_RXCHAR) == EV_RXCHAR)
		{
			while((rxd_len = pPort->rd_com((LPSTR)rxdata, G370N_MAXBLOCK)) > 0)			
			{
				Sleep(1);
				// debug
				//TRACE("\n\t received byte (%d): \n", rxd_len);
				//TRACE("\n\t received byte (%d): ", rxd_len);
				//for (int n = 0; n < rxd_len; ++n)	TRACE("0x%02X, ", rxdata[n]);
				//TRACE("\n");

				


				////--------------------------------------------------
				////	check the length of remained and received data,
				////		and concatenate remained data with currently received data
				////--------------------------------------------------
				if (rxd_len_rem == 0)
				{
					rxd_len_con = rxd_len;
					memset((void*)rxdata_con, 0, G370N_MAXBLOCK2);
					memcpy((void*)rxdata_con, (void*)rxdata, rxd_len);

					rxd_len = 0;
					memset((void*)rxdata, 0, G370N_MAXBLOCK);

					rxd_len_rem = 0;
					memset((void*)rxdata_rem, 0, G370N_MAXBLOCK);
				}
				else
				{
					rxd_len_con = rxd_len + rxd_len_rem;
					memset((void*)rxdata_con, 0, G370N_MAXBLOCK2);
					memcpy((void*)rxdata_con, (void*)rxdata_rem, rxd_len_rem);
					memcpy((void*)(rxdata_con + rxd_len_rem), (void*)rxdata, rxd_len);

					rxd_len = 0;
					memset((void*)rxdata, 0, G370N_MAXBLOCK);

					rxd_len_rem = 0;
					memset((void*)rxdata_rem, 0, G370N_MAXBLOCK);
				}

				// rxdata_con[] index to indicate the start and the end position of a packet
				sidx = 0;	// start index
				eidx = 0;	// end index

				// debug
				//TRACE("\t rxdata_con(%d): \n", rxd_len_con);
				//TRACE("\t rxdata_con(%d): ", rxd_len_con);
				//for (int n = 0; n < rxd_len_con; ++n)	TRACE("0x%02X, ", rxdata_con[n]);
				//TRACE("\n\n");


				while (sidx < rxd_len_con)
				{
					ADDR = rxdata_con[sidx];
					DLMT = rxdata_con[sidx + G370N_RSP_LEN - 1];	// to check whether this packet is respond message or not

					if (ADDR == G370N_BURST_CMD_L)// && 
					{
						// remained data length  = rxd_len_con - (sidx+1)
						if ((rxd_len_con - sidx) < (BURST_PKT_LEN - 2))	// -2 for check sum. single packet length from 0x80~0x0D < BURST_PKT_LEN : almostly valid data.
						{
							//printf(">> BURST: too short, wait next data ...\n");
							// move rxdata_con to rxdata_rem and then reset rxdata & rxdata_con
							rxd_len_rem = rxd_len_con - sidx;
							memset((void*)rxdata_rem, 0, G370N_MAXBLOCK);
							memcpy((void*)rxdata_rem, (void*)(&rxdata_con[sidx]), rxd_len_rem);

							rxd_len_con = 0;
							memset((void*)rxdata_con, 0, G370N_MAXBLOCK2);

							rxd_len = 0;
							memset((void*)rxdata, 0, G370N_MAXBLOCK);

							// to next receive sensor data
							sidx = 0; eidx = 0;
							break;
						}
						else
						{
							//printf(">> BURST: enough, parse data ...\n");
							// here, rxdata_con has a length longer than or equal to BURST_PKT_LEN
							// find delimiter 0x0D and check the length from address 0x80 (sidx) to delimiter 0x0D (eidx)							
							int k = 0;
							for (k = sidx + (BURST_PKT_LEN - 1); k > sidx; --k)
								//for (k = sidx+1; k < rxd_len_con; ++k)
							{
								if (rxdata_con[k] == G370N_DELIMITER)
								{
									// in this case, packet = {rxdata_con[sidx]~rxdata_con[k]} with delimiter
									eidx = k;
									memset((void*)rx_brst, 0, G370N_MAXBRST_LEN);
									memcpy((void*)rx_brst, (void*)(&rxdata_con[sidx]), eidx - sidx + 1);
									//printf(">> BURST(%2d): packet found w/ delimiter\n", eidx - sidx + 1);

									////------------------------------////
									////							  ////
									//// Valid BURST Data Packet Here ////
									////							  ////
									////------------------------------////
									memset((void*)&rxpkt, 0, sizeof(G370_BRST));
									int cur_burst_len = eidx - sidx + 1;

									
									if (pNtrip->isgpsupdated)
									{
										pNtrip->isgpsupdated = 0;
										sync_cnt = 0;

										if (pPort->m_fpIMU == NULL)
										{
											pPort->m_fpIMU = fopen("log_IMU.txt", "wt");
											CString str = "OPEN IMU MEASUREMENT FILE";
											pNtrip->m_pInfo->pList->AddString(str);
											pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);											
										}

										if (pNtrip->m_pInfo->engage_nav)
										{
											//if (pNtrip->imu_sts == IMU_STS_INIT_NOT_ENGAGE)
											if ((pNtrip->imu_sts & IMU_STS_INIT_SUM) == 0)
											{
												pNtrip->imu_sts |= IMU_STS_INIT_SUM;
												CString str = "IMU_STS_INIT_SUM";
												pNtrip->m_pInfo->pList->AddString(str);
												pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
											}
										}									
									}


									////----------------------------------------
									////	ND flag
									////	ND - RO(Range Over): When at least one over range condition is detected in RANGE_OVER[0x0C(W0)], this flag is set
									////----------------------------------------
									ND = rx_brst[1];
									fRO = ND & 0x01;
									//if (fRO)
									//{
									//	TRACE("\n\tERROR: RANGE OVER ...\n");
									//}


									////----------------------------------------
									////	EA flag
									////	EA - RTD(Run Time Diag): When at least one abnormality is detected in the run time self - diagnosis result(RT_DIAG[0x2B(W0)]), this flag is set to “1”.
									////	EA - EA(All Error): When at least one failure is found in the diagnostic result(DIAG_STAT[0x04(W0)]), the flag is set to “1”(failure occurred).
									////----------------------------------------
									EA = rx_brst[2];
									fEA = EA & 0x01;
									fRTD = EA & 0x02;
									//if (fEA)
									//{
									//	TRACE("\n\tERROR: ALL ERROR ...\n");
									//}
									//if (fRTD)
									//{
									//	TRACE("\n\tERROR: RUN TIME DIAG ...\n");
									//}


									////----------------------------------------
									////	Checksum Comparison
									////----------------------------------------
									// checksum host calculated
									chksm = 0;
									for (int n = 1; n < BURST_PKT_LEN - 4; n += 2)
									{
										chksm += (unsigned int)((rx_brst[n] << 8) | rx_brst[n + 1]);
									}
									chksm &= 0xFFFF;	//(lower 2 bytes)
									// checksum sensor transmited
									rxpkt.checksum = (rx_brst[BURST_PKT_LEN - 3] << 8) | rx_brst[BURST_PKT_LEN - 2];
									//if (chksm != rxpkt.checksum)
									//{
									//	if (cur_burst_len == BURST_PKT_LEN)
									//		TRACE(">> warning: checksum mimatch, packet len = %d\n", cur_burst_len);
									//}



									////----------------------------------------
									////	Now parse each data from the packet
									////----------------------------------------
									rxpkt.fTemp = (float)(((short)((rx_brst[3] << 8) | rx_brst[4]) - 2634) * SF_TEMP + 25);	// temperature	// celcius

									rxpkt.fGyro[0] = (float)((short)((rx_brst[5] << 8) | rx_brst[6]) * SF_GYRO);	// X-GYRO	// dps												
									rxpkt.fGyro[1] = (float)((short)((rx_brst[7] << 8) | rx_brst[8]) * SF_GYRO);	// Y-GYRO	// dps						
									rxpkt.fGyro[2] = (float)((short)((rx_brst[9] << 8) | rx_brst[10]) * SF_GYRO);	// Z-GYRO	// dps
									rxpkt.fAccl[0] = (float)((short)((rx_brst[11] << 8) | rx_brst[12]) * SF_ACCL);	// X-ACCL	// mG						
									rxpkt.fAccl[1] = (float)((short)((rx_brst[13] << 8) | rx_brst[14]) * SF_ACCL);	// Y-ACCL	// mG
									rxpkt.fAccl[2] = (float)((short)((rx_brst[15] << 8) | rx_brst[16]) * SF_ACCL);	// Z-ACCL	// mG

									//rxpkt.fGyro[0] = (float)((short)((rx_brst[5] << 8) | rx_brst[6]) * (SF_GYRO / fs));		// X-GYRO	// dps												
									//rxpkt.fGyro[1] = (float)((short)((rx_brst[7] << 8) | rx_brst[8]) * (SF_GYRO / fs));		// Y-GYRO	// dps						
									//rxpkt.fGyro[2] = (float)((short)((rx_brst[9] << 8) | rx_brst[10]) * (SF_GYRO / fs));	// Z-GYRO	// dps
									//rxpkt.fAccl[0] = (float)((short)((rx_brst[11] << 8) | rx_brst[12]) * (SF_ACCL / fs));	// X-ACCL	// mG						
									//rxpkt.fAccl[1] = (float)((short)((rx_brst[13] << 8) | rx_brst[14]) * (SF_ACCL / fs));	// Y-ACCL	// mG
									//rxpkt.fAccl[2] = (float)((short)((rx_brst[15] << 8) | rx_brst[16]) * (SF_ACCL / fs));	// Z-ACCL	// mG

									rxpkt.fGyro[0] = rxpkt.fGyro[0] * d2r;	// rps
									rxpkt.fGyro[1] = rxpkt.fGyro[1] * d2r;	// rps
									rxpkt.fGyro[2] = rxpkt.fGyro[2] * d2r;	// rps
									rxpkt.fAccl[0] = rxpkt.fAccl[0] * 9.8 / 1000.0;	// m/s^2
									rxpkt.fAccl[1] = rxpkt.fAccl[1] * 9.8 / 1000.0; // m/s^2
									rxpkt.fAccl[2] = rxpkt.fAccl[2] * 9.8 / 1000.0; // m/s^2

									rxpkt.cnt = (unsigned int)((rx_brst[17] << 8) | rx_brst[18]);	// Sensor Sampling Count
																		
									
									////------------------------------
									////	time (sampling period) check
									////------------------------------
									if (0)
									{
										if (_tstart == 0)
										{
											TIME_START(_tstart, _freq);
										}
										TIME_ELAPSED(_tstart, _tend, _telapsed, _freq);
									}									
									

									////------------------------------
									////	Now save and/or transmit imu data
									////------------------------------
									if (pNtrip->cur_gpst >= 0 && pNtrip->pre_gpst >= 0)
									{
										if (chksm != rxpkt.checksum)	// DEBUG only
										{
											//TRACE("BURST _ERR_(%2d, %u): %10u, %12.5f, %12.5f, %12.5f, %12.5f, %12.5f, %12.5f, %7.2f, %8.6f, (%u != %u)\n",
											//	cur_burst_len, ncounter++,
											//	rxpkt.cnt,
											//	rxpkt.fGyro[0], rxpkt.fGyro[1], rxpkt.fGyro[2],
											//	rxpkt.fAccl[0], rxpkt.fAccl[1], rxpkt.fAccl[2],
											//	rxpkt.fTemp,
											//	(double)_telapsed / TIME_UNIT,
											//	chksm, rxpkt.checksum);
										}
										else
										{
											if (pPort->m_fpIMU != NULL)
											{
												//// record measurement												
												//fprintf(pPort->m_fpIMU, "%12.2f, %3d, %12.5f, %12.5f, %12.5f, %12.5f, %12.5f, %12.5f, %7.2f, %10u, %10u\n",
												fprintf(pPort->m_fpIMU, "%12.2f\t%3d\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%7.2f\t%10u\t%6u\n",
													(double)(pNtrip->cur_gpst * 0.001),
													sync_cnt,
													rxpkt.fGyro[0], rxpkt.fGyro[1], rxpkt.fGyro[2],
													rxpkt.fAccl[0], rxpkt.fAccl[1], rxpkt.fAccl[2],
													rxpkt.fTemp,
													ncounter++,
													rxpkt.cnt);


												if (pNtrip->m_pInfo->engage_nav)
												{
													//------------------------------------
													// copy imu data struct
													//------------------------------------
													pNtrip->rawimu.pgps_time = pNtrip->rawimu.gps_time;
													
													//pNtrip->rawimu.cnt_frm_epoch = sync_cnt;
													pNtrip->rawimu.cnt_frm_epoch = (double)sync_cnt / pNtrip->m_pInfo->dFs_IMU;	// sync_cnt is 0 base index
													pNtrip->rawimu.gps_time = (double)(pNtrip->cur_gpst * 0.001) + pNtrip->rawimu.cnt_frm_epoch;													
													pNtrip->rawimu.gps_stime = pNtrip->rawimu.gps_time;	// not used
													pNtrip->rawimu.ce_time = 0;	// not used													

													pNtrip->rawimu.fb[0] = rxpkt.fAccl[0];
													pNtrip->rawimu.fb[1] = rxpkt.fAccl[1];
													pNtrip->rawimu.fb[2] = rxpkt.fAccl[2];
													pNtrip->rawimu.rbib[0] = rxpkt.fGyro[0];
													pNtrip->rawimu.rbib[1] = rxpkt.fGyro[1];
													pNtrip->rawimu.rbib[2] = rxpkt.fGyro[2];
													

													//// GPS/IMU EKF
													if (pNtrip->imu_sts & IMU_STS_INIT_SUM)
													{
														//// accumulation
														for (int ss = 0; ss < 3; ++ss)
														{
															pNtrip->axyzsum[ss] = pNtrip->axyzsum[ss] + rxpkt.fAccl[ss];	// m/s^2
															pNtrip->rxyzsum[ss] = pNtrip->rxyzsum[ss] + rxpkt.fGyro[ss];	// rps
														}
														pNtrip->Sync_idx_imu = pNtrip->Sync_idx_imu + 1;

														//// average
														if (pNtrip->Sync_idx_imu == pNtrip->num_calgn)
														{
															for (int ss = 0; ss < 3; ++ss)
															{
																pNtrip->avr.fb[ss] = pNtrip->axyzsum[ss] / pNtrip->num_calgn;
																pNtrip->avr.rbib[ss] = pNtrip->rxyzsum[ss] / pNtrip->num_calgn;
															}

															pNtrip->imu_sts |= IMU_STS_IMU_READY;
															CString str = "IMU_STS_IMU_READY";
															pNtrip->m_pInfo->pList->AddString(str);
															pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

															//// check & change status
															if ((pNtrip->imu_sts & (IMU_STS_IMU_READY | IMU_STS_GNSS_READY)) == (IMU_STS_IMU_READY | IMU_STS_GNSS_READY))
															{
																if (!(pNtrip->imu_sts & IMU_STS_COARSE_ALIGN))
																{
																	pNtrip->imu_sts |= IMU_STS_COARSE_ALIGN;
																	CString str = "IMU_STS_COARSE_ALIGN - IMU Sensor";
																	pNtrip->m_pInfo->pList->AddString(str);
																	pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
																}
															}
														} // end if (pNtrip->Sync_idx_imu == pNtrip->num_calgn)
													} // end if (pNtrip->imu_sts & IMU_STS_INIT_SUM)

													if (pNtrip->imu_sts & IMU_STS_EKF && pNtrip->ini_imu_flag == 0)
													{														
														if (pNtrip->rawimu.gps_time >= pNtrip->gps_sol.gps_time && pNtrip->rawimu.gps_time < pNtrip->gps_sol.gps_time + 1)
														//if (pNtrip->rawimu.gps_time >= pNtrip->gps_sol.gps_time)
														{
															//if (pNtrip->rawimu.gps_time != pNtrip->gps_sol.gps_time)
															if ((pNtrip->rawimu.gps_time != pNtrip->gps_sol.gps_time) || (!pNtrip->m_is_GPS_available))
															{
																/////////////////// SDINS Algorithm ///////////////////
																pNtrip->pps.gps_time = pNtrip->rawimu.gps_time;
																pNtrip->avr.gps_time = pNtrip->rawimu.gps_time;
																brm_avr_imu(&(pNtrip->h), &(pNtrip->rawimu), &(pNtrip->avr), pNtrip->Sync_idx_imu, 0);	// 측정치 바이어스 제거
																do_hspd_sdins_in_rx_seq(&(pNtrip->h), &(pNtrip->m), &(pNtrip->pps), &(pNtrip->avr));	// 자세계산																
																////////////////// SDINS Algorithm finish ///////////////////
																
																double tmpc, RPY[3], relpos[3];
																tmpc = 1 - pow(pNtrip->h.c[6], 2.0);
																tmpc = sqrt(tmpc);
																RPY[0] = atan2(pNtrip->h.c[7], pNtrip->h.c[8]);
																RPY[1] = atan2(-pNtrip->h.c[6], tmpc);
																RPY[2] = atan2(pNtrip->h.c[3], pNtrip->h.c[0])*r2d;
																if (RPY[2] < 0)
																	RPY[2] = RPY[2] + 360;

																if (pNtrip->fp_inssol != NULL)
																{
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->rawimu.gps_time);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.gps_time);

																	fprintf(pNtrip->fp_inssol, " %20.10f", RPY[0] * r2d); // 3
																	fprintf(pNtrip->fp_inssol, " %20.10f", RPY[1] * r2d);
																	fprintf(pNtrip->fp_inssol, " %20.10f", RPY[2]);

																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->m.lat*r2d); // 6
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->m.lon*r2d);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->m.hgt);

																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.lat); // 9
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.lon);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.hgt);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.hd);

																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.vel_horz);	//13
																	fprintf(pNtrip->fp_inssol, "%20.10f", sqrt(pNtrip->m.v[0] * pNtrip->m.v[0] + pNtrip->m.v[1] * pNtrip->m.v[1]));

																	fprintf(pNtrip->fp_inssol, " 1 \n");
																}																
															}
															else if ((pNtrip->rawimu.gps_time == pNtrip->gps_sol.gps_time) && (pNtrip->m_is_GPS_available))
															{
																/////////////////// SDINS Algorithm ///////////////////
																pNtrip->pps.gps_time = pNtrip->rawimu.gps_time;
																pNtrip->avr.gps_time = pNtrip->rawimu.gps_time;

																brm_avr_imu(&(pNtrip->h), &(pNtrip->rawimu), &(pNtrip->avr), pNtrip->Sync_idx_imu, 0); // 측정치 바이어스 제거

																do_hspd_sdins_in_rx_seq(&(pNtrip->h), &(pNtrip->m), &(pNtrip->pps), &(pNtrip->avr)); // 자세계산
																// 자세계산 함수 내부에서 속도계산 함수(do_mspd_sdins_in_rx_seq_lvarm)를 호출함
																////////////////// SDINS Algorithm finish ///////////////////


																/////////////////// GPS_INS Integration ///////////////////
																///////////// <------------------------- GPS/INS sync 확인 후 GPS post solution 획득
																pNtrip->pps.gpsxyz[0] = pNtrip->gps_sol.ecefxyz[0];
																pNtrip->pps.gpsxyz[1] = pNtrip->gps_sol.ecefxyz[1];
																pNtrip->pps.gpsxyz[2] = pNtrip->gps_sol.ecefxyz[2];

																pNtrip->pps.lat = pNtrip->gps_sol.lat;
																pNtrip->pps.lon = pNtrip->gps_sol.lon;
																pNtrip->pps.hgt = pNtrip->gps_sol.hgt;

																pNtrip->pps.gpsvel[0] = pNtrip->gps_sol.vel[0];
																pNtrip->pps.gpsvel[1] = pNtrip->gps_sol.vel[1];
																pNtrip->pps.gpsvel[2] = pNtrip->gps_sol.vel[2];

																tframe_do_at_gps_tight_rx_in_rx_seq(&(pNtrip->h), &(pNtrip->m), &(pNtrip->pps), &(pNtrip->avr), &(pNtrip->gps_sol), 1);  // GPS+INS 결합							

																pNtrip->gps_sol.delay = pNtrip->pps.delay;

																double tmpc, RPY[3], relpos[3];

																tmpc = 1 - pow(pNtrip->h.c[6], 2.0);
																tmpc = sqrt(tmpc);

																RPY[0] = atan2(pNtrip->h.c[7], pNtrip->h.c[8]);
																RPY[1] = atan2(-pNtrip->h.c[6], tmpc);
																RPY[2] = atan2(pNtrip->h.c[3], pNtrip->h.c[0])*r2d;
																if (RPY[2] < 0)
																	RPY[2] = RPY[2] + 360;

																if (pNtrip->fp_inssol != NULL)
																{
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->rawimu.gps_time);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.gps_time);

																	fprintf(pNtrip->fp_inssol, " %20.10f", RPY[0] * r2d); // 3
																	fprintf(pNtrip->fp_inssol, " %20.10f", RPY[1] * r2d);
																	fprintf(pNtrip->fp_inssol, " %20.10f", RPY[2]);

																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->m.lat*r2d); // 6
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->m.lon*r2d);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->m.hgt);

																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.lat); // 9
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.lon);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.hgt);
																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.hd);

																	fprintf(pNtrip->fp_inssol, " %20.10f", pNtrip->gps_sol.vel_horz);	//13

																	fprintf(pNtrip->fp_inssol, "%20.10f", sqrt(pNtrip->m.v[0] * pNtrip->m.v[0] + pNtrip->m.v[1] * pNtrip->m.v[1]));

																	fprintf(pNtrip->fp_inssol, " 0 \n");
																}


																//// update list box
																//sprintf(szlog, "%12.2f, %3d,%3d, %12.7f, %13.7f, %9.3f, %.3f, %.5f\n",
																sprintf(szlog, "%12.2f, %12.7f, %13.7f, %9.3f, %.3f, %.5f\n",
																	pNtrip->rawimu.gps_time,
																	//(int)pvt2.fixType, (int)pvt2.flags,
																	pNtrip->m.lat*r2d, pNtrip->m.lon*r2d, pNtrip->m.hgt, pNtrip->gps_sol.hd,
																	sqrt(pNtrip->m.v[0] * pNtrip->m.v[0] + pNtrip->m.v[1] * pNtrip->m.v[1]));
																pNtrip->m_pInfo->pList->AddString((LPCTSTR)(szlog));
																pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
															}
														}
													} // end if (pNtrip->imu_sts & IMU_STS_EKF && pNtrip->ini_imu_flag == 0)

												} // end if (pNtrip->m_pInfo->engage_nav)												
											} // end if (pPort->m_fpIMU != NULL)
										}
									}								

									//
									if (pPort->m_fpIMU != NULL)
										++sync_cnt;

									sidx = eidx + 1;
									break;
								}
							}// end for (k = sidx+1; k <= BURST_PKT_LEN; ++k)
							if (k == sidx)
							{
								// in this case, rxdata_con[1~end] has not address byte nor delimiter byte
								// so skip rxdata_con[] (unknown data... although rxdata_con[sidx] = 0x80)
								//
								if ((rxd_len_con - sidx) < BURST_PKT_LEN)
								{
									// wait next data stream
									//TRACE(">> BURST: wait next data ... rxd_len_con - sidx = %d\n", (rxd_len_con - sidx));

									// move rxdata_con to rxdata_rem and then reset rxdata & rxdata_con

									rxd_len_rem = rxd_len_con - sidx;
									memset((void*)rxdata_rem, 0, G370N_MAXBLOCK);
									memcpy((void*)rxdata_rem, (void*)(&rxdata_con[sidx]), rxd_len_rem);

									rxd_len_con = 0;
									memset((void*)rxdata_con, 0, G370N_MAXBLOCK2);

									rxd_len = 0;
									memset((void*)rxdata, 0, G370N_MAXBLOCK);

									// to next receive sensor data
									sidx = 0; eidx = 0;
									break;
								}
								else
								{
									//TRACE("\t\t unknown data (%d): \n", (rxd_len_con - sidx));
									//TRACE("\t\t unknown data (%d): ", (rxd_len_con - sidx));
									//for (int n = sidx; n < rxd_len_con; ++n)	TRACE("0x%02X, ", rxdata_con[n]);
									//TRACE("\n\n");
									//
									sidx += rxd_len_con;
									eidx = sidx - 1;
									rxd_len_con = 0;
									memset((void*)rxdata_con, 0, G370N_MAXBLOCK2);
								}
							}
						} // end if-else if ((rxd_len_con - sidx) < (BURST_PKT_LEN - 2))
					} // end  if (ADDR == G370N_BURST_CMD_L)// && 
					else if ((ADDR == G370N_RR_WIN_CTRL_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						if (LSB == G370N_RV_WIN_1)
						{
							WND = G370N_RV_WIN_1;
							//TRACE(">> RESPOND : %s\n", "set Window 1");
						}
						else
						{
							WND = G370N_RV_WIN_0;
							//TRACE(">> RESPOND : %s\n", "set Window 0");
						}
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_1) && (ADDR == G370N_RR_GLOB_CMD_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						if (!(MSB & G370N_RV_NOTREADY))
						{
							//TRACE(">> RESPOND : %s\n", "is READY");
							pPort->fstsimu |= G370N_STS_NOTREADY0;
						}
						else
						{
							//TRACE(">> RESPOND : %s\n", "is NOT READY ....");
							pPort->fstsimu &= ~G370N_STS_NOTREADY0;
						}
						if (!(LSB & G370N_RV_SOFTRST))
						{
							//TRACE(">> RESPOND : %s\n", "is software reset");
							pPort->fstsimu |= G370N_STS_SOFTRST0;
						}
						else
						{
							//TRACE(">> RESPOND : %s\n", "is software reset finished or... not...");
							pPort->fstsimu &= ~G370N_STS_SOFTRST0;
						}
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_0) && (ADDR == G370N_RR_MODE_CTRL_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						if ((MSB & G370N_RV_MODE_STAT))
						{
							// configuration mode
							//TRACE(">> RESPOND : %s\n", "configuration mode");
							pPort->fstsimu |= G370N_STS_CONFIGMODE;
							pPort->fstsimu &= ~G370N_STS_SMPLMODE;
						}
						else
						{
							// sampling mode
							//TRACE(">> RESPOND : %s\n", "sampling mode");
							pPort->fstsimu &= ~G370N_STS_CONFIGMODE;
							pPort->fstsimu |= G370N_STS_SMPLMODE;
							pPort->fstsimu |= G370N_STS_RUNNING;
							//rxd_len = rxd_len_con - (eidx + 1);
							//memset((void*)rxdata, 0, G370N_MAXBLOCK);
							//memcpy((void*)rxdata, (void*)(&rxdata_con[eidx + 1]), rxd_len);

							//rxd_len_rem = 0;
							//memset((void*)rxdata_rem, 0, G370N_MAXBLOCK);

							//break;
						}
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_1) && (ADDR == G370N_RR_UART_CTRL_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						if (LSB & G370N_RV_UART_AUTO)
						{
							//TRACE(">> RESPOND : %s\n", "UART Auto Mode = ON");
							pPort->fstsimu |= G370N_STS_UART_AUTO;
						}
						else
						{
							//TRACE(">> RESPOND : %s\n", "UART Auto Mode = OFF");
							pPort->fstsimu &= ~G370N_STS_UART_AUTO;
						}
						if (LSB & G370N_RV_AUTO_START)
						{
							//TRACE(">> RESPOND : %s\n", "Auto Start = ON");
							pPort->fstsimu |= G370N_STS_AUTO_START;
						}
						else
						{
							//TRACE(">> RESPOND : %s\n", "Auto Start = OFF");
							pPort->fstsimu &= ~G370N_STS_AUTO_START;
						}

						//if ((MSB & 0x03) == G370N_RV_UARTBR_460K);	//TRACE(">> RESPOND : %s = %s\n", "BAUD_RATE", "460.8 kbps");
						//if ((MSB & 0x03) == G370N_RV_UARTBR_230K);	//TRACE(">> RESPOND : %s = %s\n", "BAUD_RATE", "230.4 kbps");
						//if ((MSB & 0x03) == G370N_RV_UARTBR_921K);	//TRACE(">> RESPOND : %s = %s\n", "BAUD_RATE", "921.6 kbps");
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_0) && (ADDR == G370N_RR_DIAG_STAT_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						if (!(LSB & G370N_RV_HARD_ERR))
						{
							//TRACE(">> RESPOND : %s\n", "hardware is okay");
							pPort->fstsimu |= G370N_STS_HARDERR00;
						}
						else
						{
							//TRACE(">> RESPOND : %s\n", "hardware error occurs...");
							pPort->fstsimu &= ~G370N_STS_HARDERR00;
						}
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_1) && (ADDR == G370N_RR_SMPL_CTRL_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						pPort->fstsimu |= G370N_STS_SMPLRATE;

						if (MSB == G370N_RV_FS_2000)		smpl_rate = 2000.0;
						else if (MSB == G370N_RV_FS_1000)	smpl_rate = 1000.0;
						else if (MSB == G370N_RV_FS_500)	smpl_rate = 500.0;
						else if (MSB == G370N_RV_FS_250)	smpl_rate = 250.0;
						else if (MSB == G370N_RV_FS_125)	smpl_rate = 125.0;
						else if (MSB == G370N_RV_FS_62)		smpl_rate = 62.5;
						else if (MSB == G370N_RV_FS_31)		smpl_rate = 31.25;
						else if (MSB == G370N_RV_FS_15)		smpl_rate = 15.625;
						else if (MSB == G370N_RV_FS_400)	smpl_rate = 400.0;
						else if (MSB == G370N_RV_FS_200)	smpl_rate = 200.0;
						else if (MSB == G370N_RV_FS_100)	smpl_rate = 100.0;
						else if (MSB == G370N_RV_FS_80)		smpl_rate = 80.0;
						else if (MSB == G370N_RV_FS_50)		smpl_rate = 50.0;
						else if (MSB == G370N_RV_FS_40)		smpl_rate = 40.0;
						else if (MSB == G370N_RV_FS_25)		smpl_rate = 25.0;
						else if (MSB == G370N_RV_FS_20)		smpl_rate = 20.0;

						//TRACE(">> RESPOND : %s = %.3f\n", "sampling frequency", smpl_rate);
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_1) && (ADDR == G370N_RR_FILT_CTRL_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						pPort->fstsimu |= G370N_STS_FILTER;

						if ((LSB & 0x1F) == G370N_RV_MAF_TS_0)
						{
							filter_type = eMAF;	tap_size = 0;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_2)
						{
							filter_type = eMAF;	tap_size = 2;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_4)
						{
							filter_type = eMAF;	tap_size = 4;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_8)
						{
							filter_type = eMAF;	tap_size = 8;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_16)
						{
							filter_type = eMAF;	tap_size = 16;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_32)
						{
							filter_type = eMAF;	tap_size = 32;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_64)
						{
							filter_type = eMAF;	tap_size = 64;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						else if ((LSB & 0x1F) == G370N_RV_MAF_TS_128)
						{
							filter_type = eMAF;	tap_size = 128;
							//TRACE(">> RESPOND : %s, %s = %d\n", "MAF", "Tap", tap_size);
						}
						//TRACE(">> RESPOND : %s = 0x%02X\n", "filter selection", (LSB & 0x1F));
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
						}
					else if ((WND == G370N_RV_WIN_1) && (ADDR == G370N_RR_BRST_CTRL1_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						pPort->fstsimu |= G370N_STS_BURST1;

						ar2[eFLAG][0] = (MSB & 0x80) ? 1 : 0;
						ar2[eTEMP][0] = (MSB & 0x40) ? 1 : 0;
						ar2[eGYRO][0] = (MSB & 0x20) ? 1 : 0;
						ar2[eACCL][0] = (MSB & 0x10) ? 1 : 0;
						ar2[eDLTA][0] = (MSB & 0x08) ? 1 : 0;
						ar2[eDLTV][0] = (MSB & 0x04) ? 1 : 0;
						ar2[eGPIO][0] = (LSB & 0x04) ? 1 : 0;
						ar2[eSCNT][0] = (LSB & 0x02) ? 1 : 0;
						ar2[eCKSM][0] = (LSB & 0x01) ? 1 : 0;

						//if (MSB & 0x80)	TRACE(">> RESPOND : %s\n", "FLAG_OUT");
						//if (MSB & 0x40)	TRACE(">> RESPOND : %s\n", "TEMP_OUT");
						//if (MSB & 0x20)	TRACE(">> RESPOND : %s\n", "GYRO_OUT");
						//if (MSB & 0x10)	TRACE(">> RESPOND : %s\n", "ACCL_OUT");
						//if (MSB & 0x08)	TRACE(">> RESPOND : %s\n", "DLTA_OUT");
						//if (MSB & 0x04)	TRACE(">> RESPOND : %s\n", "DLTV_OUT");
						//if (LSB & 0x04)	TRACE(">> RESPOND : %s\n", "GPIO_OUT");
						//if (LSB & 0x02)	TRACE(">> RESPOND : %s\n", "COUNT_OUT");
						//if (LSB & 0x01)	TRACE(">> RESPOND : %s\n", "CHKSM_OUT");

						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else if ((WND == G370N_RV_WIN_1) && (ADDR == G370N_RR_BRST_CTRL2_L) && (DLMT == G370N_DELIMITER))
					{
						memset((void*)rx_rsp, 0, G370N_RSP_LEN);
						memcpy((void*)rx_rsp, (void*)(rxdata_con + sidx), G370N_RSP_LEN);
						eidx += (eidx == 0) ? (G370N_RSP_LEN - 1) : G370N_RSP_LEN;
						MSB = rx_rsp[1]; LSB = rx_rsp[2];
						pPort->fstsimu |= G370N_STS_BURST2;
						ar2[eFLAG][1] = 2;	// 16 bit fixed
						ar2[eTEMP][1] = (MSB & 0x40) ? 4 : 2;	// 32 bits (2 4 bytes) or 16 bits (2 bytes)
						ar2[eGYRO][1] = (MSB & 0x20) ? 4 : 2;	// 32 bits (2 4 bytes) or 16 bits (2 bytes)
						ar2[eACCL][1] = (MSB & 0x10) ? 4 : 2;	// 32 bits (2 4 bytes) or 16 bits (2 bytes)
						ar2[eDLTA][1] = (MSB & 0x08) ? 4 : 2;	// 32 bits (2 4 bytes) or 16 bits (2 bytes)
						ar2[eDLTV][1] = (MSB & 0x04) ? 4 : 2;	// 32 bits (2 4 bytes) or 16 bits (2 bytes)
						ar2[eGPIO][1] = 2;	// 16 bit fixed
						ar2[eSCNT][1] = 2;	// 16 bit fixed
						ar2[eCKSM][1] = 2;	// 16 bit fixed

						ar2[eGYRO][1] *= 3;	// X, Y, Z axis
						ar2[eACCL][1] *= 3;
						ar2[eDLTA][1] *= 3;
						ar2[eDLTV][1] *= 3;

						//TRACE(">> RESPOND : %s = %s\n", "TEMP", ((MSB & 0x40) ? "32-bit" : "16-bit"));
						//TRACE(">> RESPOND : %s = %s\n", "GYRO", ((MSB & 0x20) ? "32-bit" : "16-bit"));
						//TRACE(">> RESPOND : %s = %s\n", "ACCL", ((MSB & 0x10) ? "32-bit" : "16-bit"));
						//TRACE(">> RESPOND : %s = %s\n", "DLTA", ((MSB & 0x08) ? "32-bit" : "16-bit"));
						//TRACE(">> RESPOND : %s = %s\n", "DLTV", ((MSB & 0x04) ? "32-bit" : "16-bit"));
						sidx += G370N_RSP_LEN;
						//TRACE("\t\t data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n\n", rx_rsp[0], rx_rsp[1], rx_rsp[2], rx_rsp[3]);
					}
					else
					{
						//TRACE(">> RESPOND : %s\n", "UNKNOWN REPOND ....");
						// what to do here???? skip one byte
						eidx = sidx;
						sidx += 1;
						//TRACE("\t\t data: 0x%02X\n\n", rxdata_con[eidx]);
					}

					////----------------------------------------------
					//// check the length of the burst data packet
					////----------------------------------------------
					if (BURST_PKT_LEN == 0)
					{
						if ((pPort->fstsimu & G370N_STS_BURST1) && (pPort->fstsimu & G370N_STS_BURST2))
						{
							pPort->fstsimu |= G370N_STS_BRSTPKTLEN;
							BURST_PKT_LEN = 0;
							for (int k = 0; k < eNTYPES; ++k)
							{
								BURST_PKT_LEN += (ar2[k][0] * ar2[k][1]);
							}
							BURST_PKT_LEN += 2;	// address 1 byte (0x80) + delimiter 1 byte (0x0D)
							//TRACE("\n >> RESPOND : burst packet size = %d\n\n", BURST_PKT_LEN);
						}
					}


					////----------------------------------------------
					//// check sensor status : configuration mode or sampling mode
					////----------------------------------------------
					if (!(pPort->fstsimu & G370N_STS_RUNNING))
					{
						if ((pPort->fstsimu & G370N_STS_BRSTPKTLEN) && (pPort->fstsimu & G370N_STS_SMPLMODE))
						{
							//TRACE("\n\n------------ SAMPLING MODE & PACKET SIZE (%d) ------------\n\n", BURST_PKT_LEN);
							pPort->fstsimu |= G370N_STS_RUNNING;
						}
						else
						{
							pPort->fstsimu &= ~G370N_STS_RUNNING;
						}
					}
				} // end while (sidx < rxd_len_con)
			}
		} // end if ((dwEvtMask & EV_RXCHAR) == EV_RXCHAR)
		else
		{
			//TRACE("<Other Event>\r\n");
		}
	} // while (pPort->fConnected)

	//TRACE("\n\n\n\n\t IMU Loop End\n");
	//TRACE("\t IMU Loop End\n");
	//TRACE("\t IMU Loop End\n");
	//TRACE("\t IMU Loop End\n\n");

	unsigned char cmd[G370N_CMD_LEN] = { 0 };
	int cnt_sts_wait = 0;
	
	////--------------------------------------------------------------
	////	1. Software Reset (UART)
	////--------------------------------------------------------------
	//TRACE("1. Software Reset (UART)\n");

	// WINDOW = 1
	//TRACE(">> WT : WIN_CTRL - WINDOW = 1\n");
	cmd[0] = G370N_WR_WIN_CTRL;
	cmd[1] = G370N_RV_WIN_1;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);

	//TRACE(">> RD : WIN_CTRL\n");
	cmd[0] = G370N_RR_WIN_CTRL_L;
	cmd[1] = G370N_CMD_RD;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);

	// software reset
	//TRACE(">> WT : GLOB_CMD_L - SOFT_RST\n");
	//cmd[0] = G370N_RR_GLOB_CMD_L;
	cmd[0] = G370N_WR_GLOB_CMD;
	cmd[1] = G370N_RV_SOFTRST;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(1600);	// wait 1600 ms

	//TRACE(">> RD : GLOB_CMD_L\n");
	cmd[0] = G370N_RR_GLOB_CMD_L;
	cmd[1] = G370N_CMD_RD;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);

	//TRACE("\n\n\n");


	////--------------------------------------------------------------
	////	2. Configuration Mode
	////--------------------------------------------------------------
	//TRACE("2. Configuration Mode\n");

	// WINDOW = 0
	//TRACE(">> WT : WIN_CTRL - WINDOW = 0\n");
	cmd[0] = G370N_WR_WIN_CTRL;
	cmd[1] = G370N_RV_WIN_0;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);

	//TRACE(">> RD : WIN_CTRL\n");
	cmd[0] = G370N_RR_WIN_CTRL_L;
	cmd[1] = G370N_CMD_RD;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);



	// move to configuration mode
	//TRACE(">> WT : MODE_CTRL_- Move to Configuration Mode\n");
	cmd[0] = G370N_WR_MODE_CTRL;
	cmd[1] = G370N_RV_MOVE_CONFIG;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);

	//TRACE(">> RD : MODE_CTRL_L\n");
	cmd[0] = G370N_RR_MODE_CTRL_L;
	cmd[1] = G370N_CMD_RD;
	cmd[2] = G370N_DELIMITER;
	pPort->wt_com((LPSTR)cmd, G370N_CMD_LEN);
	Sleep(G370N_RSP_WAIT_TIME);


	//TRACE("\n\n\n");



	if (pPort->m_fpIMU != NULL)
	{
		fclose(pPort->m_fpIMU);
		pPort->m_fpIMU = NULL;
		CString str = "CLOSE IMU MEASUREMENT FILE";
		pNtrip->m_pInfo->pList->AddString(str);
		pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);
	}

	CloseHandle(os.hEvent);
	pPort->dwThreadID = 0;
	pPort->hWatchThread = NULL;

	CString str = "IMU PORT is Closed...";
	pNtrip->m_pInfo->pList->AddString(str);
	pNtrip->m_pInfo->pList->SendMessage(WM_VSCROLL, SB_BOTTOM);

	return TRUE;
}
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


int CNTRIP::Parser(char* stmp, char *tmp)
{
	//char szdelimiter[5] = ",";
	//char szdelimiter[5] = "\t";
	tmp = strtok(NULL, "\t");

	if (tmp != NULL)
	{
		//gps_sol.gps_time = atof(tmp);	// 													//2
		tmp = strtok(NULL, ",\t");															//3
		tmp = strtok(NULL, ",\t");															//4
		tmp = strtok(NULL, ",\t");															//5
		tmp = strtok(NULL, ",\t");															//6
		tmp = strtok(NULL, ",\t");															//7
		tmp = strtok(NULL, ",\t");															//8
		tmp = strtok(NULL, ",\t");															//9

		tmp = strtok(NULL, ",\t");		gps_sol.lat = atof(tmp);		// Lat (deg)		//10
		tmp = strtok(NULL, ",\t");		gps_sol.lon = atof(tmp);		// Lon (deg)		//11
		tmp = strtok(NULL, ",\t");		gps_sol.hgt = atof(tmp);		// Hgt (m)			//12

		tmp = strtok(NULL, ",\t");															//13							
		tmp = strtok(NULL, ",\t");															//14
		tmp = strtok(NULL, ",\t");															//15

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

int	CNTRIP::GetGPSTfrmUTC(int year, int month, int day, int hour, int minute, double second, double *gpstime, int &leapsec)
{
	int   dayofw, dayofy, yr, ttlday, m, weekno;
	static  int  dinmth[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int i;

	//	leap second array
	const static int leapsec_arr[][7] = {
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
		for (m = 1; m <= (month - 1); m++)
		{
			dayofy += dinmth[m];
			if (m == 2)
			{
				if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0)
					dayofy += 1;
			}
		}
		dayofy += day;
	}

	/*  Convert day of year and year into week number and day of week */
	ttlday = 360;
	leapsec = 0;
	for (yr = 1981; yr <= (year - 1); yr++)
	{
		ttlday += 365;
		if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 == 0)
		{
			ttlday += 1;
		}
	}
	ttlday += dayofy;
	weekno = ttlday / 7;
	dayofw = ttlday - 7 * weekno;

	for (i = 0; i < MAX_LEAPSEC; i++)
	{
		if ((leapsec_arr[i][0] <= year))
		{
			if (leapsec_arr[i][1] <= month)
			{
				if (leapsec_arr[i][2] <= day)
				{
					leapsec = leapsec_arr[i][6];
					break;
				}
			}
		}
	}
	*gpstime = (second + minute * 60 + hour * 3600 + dayofw * 86400 + weekno * SECONDS_IN_WEEK + leapsec);

	return weekno;
}