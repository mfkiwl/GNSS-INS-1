// F:\NISL Solution\RTNetSolution\SRC\NTRIP\NtripSocket.cpp : 구현 파일입니다.
//

#include "../../stdafx.h"
#include "../../RTNetSolution.h"
#include "NtripSocket.h"


// CNtripSocket

CNtripSocket::CNtripSocket()
{
	m_pHwnd = NULL;
}

CNtripSocket::~CNtripSocket()
{
}


// CNtripSocket 멤버 함수


void CNtripSocket::OnClose(int nErrorCode)
{
	ASSERT( m_pHwnd != NULL );
	//::SendMessage(m_pHwnd, WM_CLOSE_SERVER, 0, 0);
	::PostMessage(m_pHwnd, WM_CLOSE_SERVER, NULL, NULL);

	CAsyncSocket::OnClose(nErrorCode);
}

void CNtripSocket::OnConnect(int nErrorCode)
{
	ASSERT( m_pHwnd != NULL );
	//접속 에러 처리. MSDN참조.
	if (0 != nErrorCode)
	{
		switch( nErrorCode )
		{
		case WSAEADDRINUSE: 
			AfxMessageBox("The specified address is in use.\n");
			break;
		case WSAEADDRNOTAVAIL: 
			AfxMessageBox("The specified address is not available from the local machine.\n");
			break;
		case WSAEAFNOSUPPORT: 
			AfxMessageBox("Addresses in the specified family cannot be used with this socket.\n");
			break;
		case WSAECONNREFUSED: 
			AfxMessageBox("The attempt to connect was forcefully rejected.\n");
			break;
		case WSAEDESTADDRREQ: 
			AfxMessageBox("A destination address is required.\n");
			break;
		case WSAEFAULT: 
			AfxMessageBox("The lpSockAddrLen argument is incorrect.\n");
			break;
		case WSAEINVAL: 
			AfxMessageBox("The socket is already bound to an address.\n");
			break;
		case WSAEISCONN: 
			AfxMessageBox("The socket is already connected.\n");
			break;
		case WSAEMFILE: 
			AfxMessageBox("No more file descriptors are available.\n");
			break;
		case WSAENETUNREACH: 
			AfxMessageBox("The network cannot be reached from this host at this time.\n");
			break;
		case WSAENOBUFS: 
			AfxMessageBox("No buffer space is available. The socket cannot be connected.\n");
			break;
		case WSAENOTCONN: 
			AfxMessageBox("The socket is not connected.\n");
			break;
		case WSAENOTSOCK: 
			AfxMessageBox("The descriptor is a file, not a socket.\n");
			break;
		case WSAETIMEDOUT: 
			AfxMessageBox("The attempt to connect timed out without establishing a connection. \n");
			break;
		default:
			TCHAR szError[256];
			wsprintf(szError, "OnConnect error: %d", nErrorCode);
			AfxMessageBox(szError);
			break;
		}
		//::SendMessage(m_pHwnd, WM_CONNECT_ERROR, NULL, WITH_ERRORCODE);
		::PostMessage(m_pHwnd, WM_CONNECT_ERROR, NULL, NULL);
	}
	else{
		//::SendMessage(m_pHwnd, WM_CONNECT_ERROR, NULL, WITHOUT_ERRORCODE);
		::PostMessage(m_pHwnd, WM_CONNECT_NOERROR, NULL, NULL);
	}

	CAsyncSocket::OnConnect(nErrorCode);
}

void CNtripSocket::OnReceive(int nErrorCode)
{
	ASSERT( m_pHwnd != NULL );
	//::SendMessage(m_pHwnd, WM_RECEIVE_DATA, 0, 0);
	::PostMessage(m_pHwnd, WM_RECEIVE_DATA, NULL, NULL);

	CAsyncSocket::OnReceive(nErrorCode);
}

void CNtripSocket::SetHwnd(HWND hWnd)
{
	m_pHwnd = hWnd;
}