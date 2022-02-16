#pragma once

#define WM_RECEIVE_DATA		WM_APP+0x01
#define WM_CLOSE_SERVER		WM_APP+0x02
#define WM_CONNECT_ERROR	WM_APP+0x03
#define WM_CONNECT_NOERROR	WM_APP+0x04

#define WITH_ERRORCODE		0x01
#define WITHOUT_ERRORCODE	0x02

class CNtripSocket : public CAsyncSocket
{
public:
	CNtripSocket();
	virtual ~CNtripSocket();

	virtual void OnClose(int nErrorCode);
	virtual void OnConnect(int nErrorCode);
	virtual void OnReceive(int nErrorCode);
	void SetHwnd(HWND hWnd);
	HWND m_pHwnd;
};


