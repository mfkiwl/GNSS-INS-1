// RTNetSolutionDlg.h : header file
//

#pragma once


#include "UITNtrip.h"
#include "afxcmn.h"





////****  using NEWMAT  ****////
////
#define NOMINMAX

#define WANT_MATH
//#define WANT_STREAM
#include "SRC/NEWMAT10D/newmatap.h"
#include "SRC/NEWMAT10D/precisio.h"

#ifdef use_namespace
using namespace NEWMAT;
#endif



#define BUFSIZE1024 (1024)



// CRTNetSolutionDlg dialog
class CRTNetSolutionDlg : public CDialog
{
// Construction
public:
	CRTNetSolutionDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_RTNETSOLUTION_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	
// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);


public:
	afx_msg void OnBnClickedBnStart();
	afx_msg void OnBnClickedBnStop();
	

	// GUI
	CButton* mc_pBnStart;
	CButton* mc_pBnStop;
	CButton* mc_pBnClose;
	CButton* mc_pBnGNSSON;
	CButton* mc_pBnConnGNSS;
	CButton* mc_pBnConnIMU;

	CComboBox* mc_pCBPortGNSS;
	CComboBox* mc_pCBPortIMU;

	CListBox* mc_pLB;


	// NTRIP
	CString mc_strIP;
	CString mc_strPort;
	CString mc_strID;
	CString mc_strPW;
	CString mc_strMP;


	// Threshold
	double m_dthres_hdop;
	double m_dthres_hacc;

	// sampling rate
	double m_dFs_GNSS;
	double m_dFs_IMU;
	double m_dTapSize_IMU;

	// port number
	//int m_nPortGNSS;
	//int m_nPortIMU;
	int m_isGNSSConnected;
	int m_isIMUConnected;	

	// thread
	CUITNtrip*	m_pUIT;
	LPARGNTRIP	m_pArg;
			
	int fsts;	// 0(not-ready, not-running), 1(ready), 2(running)

public:
	int load_options();
	afx_msg void OnBnClickedBnConnectGnss();
	afx_msg void OnBnClickedBnConnectImu();
	afx_msg void OnBnClickedBnGnsson();
	afx_msg void OnBnClickedBnClose();	
	//afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedBnPp();






	//// Post Processing
	char m_szFileGPS[1024];
	char m_szFileIMU[1024];
	char m_szFileSOL[1024];
	char m_szFileSOL2[1024];

	int run_gpsins_pp();
	int Parser(char* stmp, char *tmp, type_gps_sol& gps_sol);
	int	GetGPSTfrmUTC(int year, int month, int day, int hour, int minute, double second, double *gpstime, int &leapsec);
};

#define MAX_STRUCTSIZE	172800
#define MAX_LINE		256
#define MAX_LEAPSEC		17				//	maximum number of leapsecond array