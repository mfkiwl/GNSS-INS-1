#pragma once


#include "SRC/NTRIP/NTRIP.h"


#define WM_DLG_INIT				(WM_APP+0x0121)	// main -> UITNtrip	// call Initilaizer

#define WM_DLG_CONNECTIMU		(WM_APP+0x0122)	// main -> UITNtrip // 
#define WM_DLG_CONNECTGNSS		(WM_APP+0x0123)	// main -> UITNtrip // 
#define WM_DLG_CONNECTNTRIP		(WM_APP+0x0124)	// main -> UITNtrip	// call ConnectNtrip

#define WM_DLG_NAVSTART			(WM_APP+0x0125)
#define WM_DLG_NAVSTOP			(WM_APP+0x0126)

#define WM_DLG_TERMINATE		(WM_APP+0x0127)	// main -> UITNtrip	// call 


class CUITNtrip : public CWinThread
{
	DECLARE_DYNCREATE(CUITNtrip)

protected:
	CUITNtrip();           // 동적 만들기에 사용되는 protected 생성자입니다.
	virtual ~CUITNtrip();

public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

protected:
	DECLARE_MESSAGE_MAP()

public:
	CNTRIP*		m_pNtrip;	// << NtripSocket.h
	LPARGNTRIP	m_pArg;		// << NTRIP.h에 넘길 정보.


	afx_msg void OnWmDlgInit(WPARAM wParam, LPARAM lParam);

	afx_msg void OnWmDlgConnetIMU(WPARAM wParam, LPARAM lParam);
	afx_msg void OnWmDlgConnetGNSS(WPARAM wParam, LPARAM lParam);
	afx_msg void OnWmDlgConnetNtrip(WPARAM wParam, LPARAM lParam);

	afx_msg void OnWmDlgNavStart(WPARAM wParam, LPARAM lParam);
	afx_msg void OnWmDlgNavStop(WPARAM wParam, LPARAM lParam);

	afx_msg void OnWmDlgTerminate(WPARAM wParam, LPARAM lParam);
	

	//CCriticalSection	m_cs;
};