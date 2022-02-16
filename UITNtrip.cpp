// UITNtrip.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "RTNetSolution.h"
#include "UITNtrip.h"


// CUITNtrip

IMPLEMENT_DYNCREATE(CUITNtrip, CWinThread)

CUITNtrip::CUITNtrip()
{
	m_pArg = NULL;
	m_pNtrip = NULL;
}

CUITNtrip::~CUITNtrip()
{
}

BOOL CUITNtrip::InitInstance()
{	
	if (!AfxSocketInit())
	{
		AfxMessageBox(IDP_SOCKETS_INIT_FAILED);
		return FALSE;
	}

	if ( m_pNtrip == NULL )
	{
		m_pNtrip = (CNTRIP*) new CNTRIP();
		m_pNtrip->Create(NULL, "");
		m_pNtrip->m_Sock.SetHwnd(m_pNtrip->m_hWnd);
	}
	
	ASSERT( m_pNtrip != NULL );

	//// UI Thread에 Ntrip Window를 등록한다.
	m_pMainWnd = m_pNtrip;
	
	if (m_pArg->REQ_FLAG & REQ_SRCT)
	{
		m_pNtrip->NtripInitializer(m_pArg->pSRC, m_pArg);
	}
	else
	{
		m_pNtrip->NtripInitializer(m_pArg);
	}
		
	return TRUE;
}

int CUITNtrip::ExitInstance()
{
	//m_pNtrip->_serialIMU.fConnected = 0;
	//Sleep(10000);
	//m_pNtrip->_serialGPS.fConnected = 0;
	//Sleep(10000);

	if ( m_pNtrip != NULL )
	{
		m_pNtrip->DestroyWindow();
		m_pNtrip = NULL;
	}

	return CWinThread::ExitInstance();
}

BEGIN_MESSAGE_MAP(CUITNtrip, CWinThread)
	ON_THREAD_MESSAGE(WM_DLG_INIT, OnWmDlgInit)
	ON_THREAD_MESSAGE(WM_DLG_CONNECTIMU, OnWmDlgConnetIMU)
	ON_THREAD_MESSAGE(WM_DLG_CONNECTGNSS, OnWmDlgConnetGNSS)
	ON_THREAD_MESSAGE(WM_DLG_CONNECTNTRIP, OnWmDlgConnetNtrip)	
	ON_THREAD_MESSAGE(WM_DLG_NAVSTART, OnWmDlgNavStart)
	ON_THREAD_MESSAGE(WM_DLG_NAVSTOP, OnWmDlgNavStop)
	ON_THREAD_MESSAGE(WM_DLG_TERMINATE, OnWmDlgTerminate)
END_MESSAGE_MAP()


void CUITNtrip::OnWmDlgInit(WPARAM wParam, LPARAM lParam)
{
	m_pArg = (LPARGNTRIP)wParam;

	ASSERT(m_pArg != NULL);

	if (m_pNtrip->m_nflagNtrip != -1) 
	{
		// TODO...
	}
	else
	{
		if (m_pArg->REQ_FLAG & REQ_SRCT)
		{
			LPNTRIP_SOURCE_INFO pSRC = (LPNTRIP_SOURCE_INFO)lParam;
			ASSERT(pSRC != NULL);

			m_pNtrip->NtripInitializer(pSRC, m_pArg);
		}
		else 
		{
			m_pNtrip->NtripInitializer(m_pArg);
		}
	}
}

void CUITNtrip::OnWmDlgConnetIMU(WPARAM wParam, LPARAM lParam)
{
	m_pNtrip->ConnectIMU();
}

void CUITNtrip::OnWmDlgConnetGNSS(WPARAM wParam, LPARAM lParam)
{
	m_pNtrip->ConnectGNSS();
}

void CUITNtrip::OnWmDlgConnetNtrip(WPARAM wParam, LPARAM lParam)
{
	m_pNtrip->ConnectNtrip();
}

void CUITNtrip::OnWmDlgNavStart(WPARAM wParam, LPARAM lParam)
{
	m_pNtrip->NavStart();
}

void CUITNtrip::OnWmDlgNavStop(WPARAM wParam, LPARAM lParam)
{
	m_pNtrip->NavStop();
}

void CUITNtrip::OnWmDlgTerminate(WPARAM wParam, LPARAM lParam)
{
	//m_pNtrip->_serialIMU.fConnected = 0;
	//m_pNtrip->_serialGPS.fConnected = 0;

	m_pNtrip->_serialIMU.ClosePort();
	Sleep(1000);

	m_pNtrip->_serialGPS.ClosePort();
	Sleep(1000);
}

