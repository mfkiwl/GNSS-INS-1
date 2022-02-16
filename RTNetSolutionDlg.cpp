#pragma once
#include "stdafx.h"
#include "RTNetSolution.h"
#include "RTNetSolutionDlg.h"

#include "SRC/SUBFUNC/SUBFUNC.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#pragma warning(disable:4996)
#pragma warning(disable:4101)
#pragma warning(disable:4244)

CRTNetSolutionDlg::CRTNetSolutionDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRTNetSolutionDlg::IDD, pParent)
{	
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRTNetSolutionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CRTNetSolutionDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()	
	ON_BN_CLICKED(IDC_BN_START, &CRTNetSolutionDlg::OnBnClickedBnStart)
	ON_BN_CLICKED(IDC_BN_STOP, &CRTNetSolutionDlg::OnBnClickedBnStop)	
	ON_BN_CLICKED(IDC_BN_CONNECT_GNSS, &CRTNetSolutionDlg::OnBnClickedBnConnectGnss)
	ON_BN_CLICKED(IDC_BN_CONNECT_IMU, &CRTNetSolutionDlg::OnBnClickedBnConnectImu)
	ON_BN_CLICKED(IDC_BN_GNSSON, &CRTNetSolutionDlg::OnBnClickedBnGnsson)
	ON_BN_CLICKED(IDC_BN_CLOSE, &CRTNetSolutionDlg::OnBnClickedBnClose)	
	//ON_BN_CLICKED(IDC_BUTTON2, &CRTNetSolutionDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BN_PP, &CRTNetSolutionDlg::OnBnClickedBnPp)
END_MESSAGE_MAP()

BOOL CRTNetSolutionDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon


	////========================================================================
	////
	////========================================================================	
	GetDlgItem(IDC_BN_START)->EnableWindow(TRUE);
	GetDlgItem(IDC_BN_STOP)->EnableWindow(FALSE);	
	

	mc_pBnStart = (CButton*)GetDlgItem(IDC_BN_START);
	mc_pBnStop = (CButton*)GetDlgItem(IDC_BN_STOP);
	mc_pBnClose = (CButton*)GetDlgItem(IDC_BN_CLOSE);
	mc_pBnGNSSON = (CButton*)GetDlgItem(IDC_BN_GNSSON);
	mc_pBnConnGNSS = (CButton*)GetDlgItem(IDC_BN_CONNECT_GNSS);
	mc_pBnConnIMU = (CButton*)GetDlgItem(IDC_BN_CONNECT_IMU);

	mc_pCBPortGNSS = (CComboBox*)GetDlgItem(IDC_CB_GNSS);
	mc_pCBPortIMU = (CComboBox*)GetDlgItem(IDC_CB_IMU);
	mc_pLB = (CListBox *)GetDlgItem(IDC_LIST);

	fsts = 0;
	load_options();

	mc_pCBPortGNSS->SetCurSel(0);
	mc_pCBPortIMU->SetCurSel(0);

	m_isGNSSConnected = 0;
	m_isIMUConnected = 0;

	

	////========================================================================
	////
	////========================================================================
	m_pArg = (LPARGNTRIP) new ARGNTRIP;
	::ZeroMemory(m_pArg, sizeof(ARGNTRIP));

	m_pArg->hParent = this->m_hWnd;
	m_pArg->nModeStream = NTRIP_FILE_RECORDING;
	m_pArg->REQ_FLAG = REQ_VRS;
	m_pArg->nGNSS = NTRIP_GPS;
	
	sprintf(m_pArg->szIP, "%s", mc_strIP.GetBuffer());
	sprintf(m_pArg->szPORT, "%s", mc_strPort.GetBuffer());
	sprintf(m_pArg->szID, "%s", mc_strID.GetBuffer());
	sprintf(m_pArg->szPW, "%s", mc_strPW.GetBuffer());
	
	Matrix mxyz(3, 1);
	mxyz << -3034487.50196487 << 4068368.65380958 << 3849993.91783488;	// 배곧
	//mxyz << -3062291.12 << 4053908.64 << 3843271.71;	// 원희캐슬광교
	Matrix mllh = xyz2llh(mxyz);
	m_pArg->VRS.dLatd = mllh(1, 1)*r2d;
	m_pArg->VRS.dLond = mllh(2, 1)*r2d;
	m_pArg->VRS.dAltm = mllh(3, 1);
	sprintf(m_pArg->VRS.RTKNet, "%s", mc_strMP.GetBuffer());

	m_pArg->pList = mc_pLB;
	m_pArg->pSRC = NULL;

	//m_pArg->nPortGNSS = m_nPortGNSS;
	//m_pArg->nPortIMU = m_nPortIMU;

	m_pArg->dFs_GNSS = m_dFs_GNSS;
	m_pArg->dFs_IMU = m_dFs_IMU;
	m_pArg->dTapSize_IMU = m_dTapSize_IMU;

	m_pArg->engage_nav = 0;
	m_pArg->engage_gnss = 1;

	CString str = "GNSS OFF";
	mc_pBnGNSSON->SetWindowTextA(str);

	////========================================================================
	////
	////========================================================================
	m_pUIT = (CUITNtrip*)AfxBeginThread(
		RUNTIME_CLASS(CUITNtrip),
		THREAD_PRIORITY_NORMAL,
		0,
		CREATE_SUSPENDED);

	if (m_pUIT != NULL)
	{
		m_pUIT->m_pArg = m_pArg;
		m_pArg->dwThreadID = m_pUIT->m_nThreadID;
		m_pArg->hThread = m_pUIT->m_hThread;
		m_pUIT->ResumeThread();
	}
	   
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRTNetSolutionDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
		CDialog::OnSysCommand(nID, lParam);	
}

void CRTNetSolutionDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

HCURSOR CRTNetSolutionDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

BOOL CRTNetSolutionDlg::PreTranslateMessage(MSG* pMsg)
{
	switch(pMsg->message)
	{
		// 키가 눌렸을때
	case WM_KEYDOWN:
		//switch(pMsg->message)
		switch(pMsg->wParam)
		{
			// 리턴키 \tab
		case VK_RETURN:
			return TRUE;
			break;
			// ESC키
		case VK_ESCAPE:
			return TRUE;
			break;
		}

		break;
	}

	return CDialog::PreTranslateMessage(pMsg);
}

int CRTNetSolutionDlg::load_options()
{
	int ret = 0;

	FILE* fp = fopen("adim_options.txt", "r");
	if (fp == NULL)
	{
		// default: NGII VRS
		mc_strIP = "201.117.198.81";
		mc_strPort = "2101";
		mc_strID = "firedc05";
		mc_strPW = "ngii";
		mc_strMP = "VRS-RTCM31";

		// default: threshold
		m_dthres_hdop = 4.0;
		m_dthres_hacc = 1.0;

		// default: sampling rate
		m_dFs_GNSS = 1.0;
		m_dFs_IMU = 50.0;
		m_dTapSize_IMU = 32.0;
	}
	else
	{
		char szbuf[BUFSIZE1024] = { 0 };
		char *pstr = NULL;

		do {
			::ZeroMemory(szbuf, BUFSIZE1024);
			if (fgets(szbuf, BUFSIZE1024 - 1, fp) == NULL)
			{
				break;
			}

			pstr = NULL;
			if ((pstr = strstr(szbuf, "NTRIP IP")) != NULL)
			{
				mc_strIP.Format("%s", pstr + 10);
				mc_strIP.SetAt(mc_strIP.GetLength() - 1, 0);
			}
			else if ((pstr = strstr(szbuf, "NTRIP PORT")) != NULL)
			{
				mc_strPort.Format("%s", pstr + 12);
				mc_strPort.SetAt(mc_strPort.GetLength() - 1, 0);
			}
			else if ((pstr = strstr(szbuf, "NTRIP ID")) != NULL)
			{
				mc_strID.Format("%s", pstr + 10);
				mc_strID.SetAt(mc_strID.GetLength() - 1, 0);
			}
			else if ((pstr = strstr(szbuf, "NTRIP PW")) != NULL)
			{
				mc_strPW.Format("%s", pstr + 10);
				mc_strPW.SetAt(mc_strPW.GetLength() - 1, 0);
			}
			else if ((pstr = strstr(szbuf, "NTRIP VRS MountPoint")) != NULL)
			{
				mc_strMP.Format("%s", pstr + 22);
				mc_strMP.SetAt(mc_strMP.GetLength() - 1, 0);
			}
			else if ((pstr = strstr(szbuf, "THRES DOP")) != NULL)
			{
				m_dthres_hdop = (double)atof(pstr + 11);
			}
			else if ((pstr = strstr(szbuf, "THRES HACC")) != NULL)
			{
				m_dthres_hacc = (double)atof(pstr + 12);
			}
			else if ((pstr = strstr(szbuf, "IMU SAMPLING RATE")) != NULL)
			{
				m_dFs_IMU = (double)atof(pstr + 19);
			}
			else if ((pstr = strstr(szbuf, "IMU FIL TAP SIZE")) != NULL)
			{
				m_dTapSize_IMU = (double)atof(pstr + 18);
			}
			else if ((pstr = strstr(szbuf, "GNSS SAMPLING RATE")) != NULL)
			{
				m_dFs_GNSS = (double)atof(pstr + 20);
			}			
			else
			{
				// TODO
			}
		} while (!feof(fp));

		if (mc_strIP.GetLength() == 0)
		{
			AfxMessageBox("WARNING: NTRIP IP does not exist ...");
			ret = 1;
		}
		if (mc_strPort.GetLength() == 0)
		{
			AfxMessageBox("WARNING: NTRIP PORT does not exist ...");
			ret = 1;
		}
		if (mc_strID.GetLength() == 0)
		{
			AfxMessageBox("WARNING: NTRIP ID does not exist ...");
			ret = 1;
		}
		if (mc_strPW.GetLength() == 0)
		{
			AfxMessageBox("WARNING: NTRIP PW does not exist ...");
			ret = 1;
		}
		if (mc_strMP.GetLength() == 0)
		{
			AfxMessageBox("WARNING: NTRIP MountPoint does not exist ...");
			ret = 1;
		}
		if (m_dthres_hdop == 0)
		{
			m_dthres_hdop = 4.0;
		}
		if (m_dthres_hacc == 0)
		{
			m_dthres_hacc = 1.0;
		}
		if (m_dFs_GNSS == 0)
		{
			m_dFs_GNSS = 1.0;
		}
		if (m_dFs_IMU == 0)
		{
			m_dFs_IMU = 50.0;
		}
	}

	return ret;
}

void CRTNetSolutionDlg::OnBnClickedBnStart()
{	
	m_pArg->engage_nav = 1;
	
	// toggle
	GetDlgItem(IDC_BN_START)->EnableWindow(FALSE);
	GetDlgItem(IDC_BN_STOP)->EnableWindow(TRUE);

	::PostThreadMessage(m_pArg->dwThreadID, WM_DLG_NAVSTART, NULL, NULL);
}

void CRTNetSolutionDlg::OnBnClickedBnStop()
{	
	m_pArg->engage_nav = 0;

	// toggle
	GetDlgItem(IDC_BN_START)->EnableWindow(TRUE);
	GetDlgItem(IDC_BN_STOP)->EnableWindow(FALSE);

	::PostThreadMessage(m_pArg->dwThreadID, WM_DLG_NAVSTOP, NULL, NULL);
}

void CRTNetSolutionDlg::OnBnClickedBnConnectGnss()
{
	int n = mc_pCBPortGNSS->GetCurSel();
	m_pArg->nPortGNSS = n + 3;

	::PostThreadMessage(m_pArg->dwThreadID, WM_DLG_CONNECTGNSS, NULL, NULL);
	m_isGNSSConnected = 1;
	m_pArg->isGNSSConnected = m_isGNSSConnected;
	GetDlgItem(IDC_BN_CONNECT_GNSS)->EnableWindow(FALSE);
	
	Sleep(1000);
	::PostThreadMessage(m_pArg->dwThreadID, WM_DLG_CONNECTNTRIP, NULL, NULL);
}


void CRTNetSolutionDlg::OnBnClickedBnConnectImu()
{
	if (m_isGNSSConnected != 0)
	{
		int n = mc_pCBPortIMU->GetCurSel();
		m_pArg->nPortIMU = n + 3;

		::PostThreadMessage(m_pArg->dwThreadID, WM_DLG_CONNECTIMU, NULL, NULL);
		m_isIMUConnected = 1;
		m_pArg->isIMUConnected = m_isIMUConnected;
		GetDlgItem(IDC_BN_CONNECT_IMU)->EnableWindow(FALSE);
	}
	else
	{
		AfxMessageBox("Connect GNSS Rx first !!");
	}
}


void CRTNetSolutionDlg::OnBnClickedBnGnsson()
{
	CString str;

	if (m_pArg->engage_gnss == 1)
	{
		str = "GNSS ON";
		m_pArg->engage_gnss = 0;	// ON --> OFF
	}
	else
	{
		str = "GNSS OFF";
		m_pArg->engage_gnss = 1;	// OFF --> ON
	}
	
	mc_pBnGNSSON->SetWindowTextA(str);
}


void CRTNetSolutionDlg::OnBnClickedBnClose()
{	
	//::SendMessage(m_pArg->dwThreadID, WM_DLG_TERMINATE, NULL, NULL);
	::PostThreadMessage(m_pArg->dwThreadID, WM_DLG_TERMINATE, NULL, NULL);
	Sleep(2000);

	DWORD dwExitCode = 0;
	::PostThreadMessage(m_pArg->dwThreadID, WM_QUIT, 0, NULL);
	Sleep(2000);
	//Sleep(30000);

	//UpdateData(FALSE);
	//m_pUIT = NULL;
	//delete m_pArg;
	//m_pArg = NULL;
	//Sleep(500);
}



//
//void CRTNetSolutionDlg::OnBnClickedButton2()
//{
//	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
//}


void CRTNetSolutionDlg::OnBnClickedBnPp()
{
	// Post-Processing Here

	CFileDialog	dlg(TRUE,		
		NULL,
		0,
		/*OFN_ALLOWMULTISELECT | */OFN_DONTADDTORECENT | OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR | OFN_SHAREAWARE,
		//_T("OBS Files(*.12o)|*.12o|OBS Files(*.11o)|*.11o|OBS Files(*.10o)|*.10o|OBS Files(*.03o)|*.03o|All Files(*.*)|*.*||"),
		_T("log GPS (*gps*.txt)|*gps*.txt|All Files(*.*)|*.*||"),
		this,
		0);

	INT_PTR nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// GPS
		CString strGPS = dlg.GetPathName();
		memset((void*)m_szFileGPS, 0, 1024);
		sprintf(m_szFileGPS, "%s", strGPS.GetBuffer());
	
		// IMU
		CString strIMU = strGPS;
		int nTarget = strIMU.ReverseFind('\\');
		char* psz = strIMU.GetBuffer();
		memset((void*)m_szFileIMU, 0, 1024);
		sprintf(m_szFileIMU, "%s", psz);
		sprintf(m_szFileIMU + nTarget + 1, "%s", "log_IMU.txt");

		// SOL1
		CString strSol = strGPS;		
		psz = strSol.GetBuffer();
		memset((void*)m_szFileSOL, 0, 1024);
		sprintf(m_szFileSOL, "%s", psz);
		sprintf(m_szFileSOL + nTarget + 1, "%s", "inssol_pp.txt");
		
		// SOL2
		CString strSol2 = strGPS;
		psz = strSol2.GetBuffer();
		memset((void*)m_szFileSOL2, 0, 1024);
		sprintf(m_szFileSOL2, "%s", psz);
		sprintf(m_szFileSOL2 + nTarget + 1, "%s", "inssol2_pp.txt");
		
		TRACE("\n\n\tPP File OK\n\n");
	}

	run_gpsins_pp();	
}

int CRTNetSolutionDlg::run_gpsins_pp()
{
	CString str = "post-processing...";
	mc_pLB->AddString(str);
	mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);

	char input_file_gps[1024] = { 0 };
	char input_file_imu[1024] = { 0 };
	char output_file_inssol[1024] = { 0 };
	char output_file_inssol2[1024] = { 0 };

	sprintf(input_file_gps, "%s", m_szFileGPS);
	sprintf(input_file_imu, "%s", m_szFileIMU);
	sprintf(output_file_inssol, "%s", m_szFileSOL);
	sprintf(output_file_inssol2, "%s", m_szFileSOL2);

	int gps_read_cnt;
	int rmc_f, gga_f, vtg_f;
	int flag_gps_new, flag_init, ini_imu_flag;
	


	type_gps_sol			gps_sol;			// GPS 관련 구조체
	type_gps_sol			pre_gps_sol;		// GPS 관련 구조체
	type_raw_imu			**pprawimu;				// IMU/SPD 관련 구조체

	type_hspd_sdins			h;					// 자세 관련 구조체
	type_mspd_sdins			m;					// 속도 관련 구조체
	type_at_epoch			pps;				// 필터 관련 구조체
	type_avr_imu			avr;				// 바이어스가 제거된 측정치

	int flag_gps, flag_ins;
		
	int i, j, end_line, Sync_idx_imu;
	double axyzsum[3] = { 0.0 }, rxyzsum[3] = { 0.0 };
	int num_calgn = 20; // 초반 측정치 안정화를 위한 측정치 평균 범위 (GPS 초 단위)
	int ins_calgn = 1;
	double tmpc, RPY[3];
	double ini_gpstime = 0;



	// 정지상태
	double acc_buf[2][100] = { 0 };
	double hacc[100] = { 0 };
	double hacc_std = 0;
	double hacc_mean = 0;
	double stop_sts = -1;

	double dhacc_sum = .0;
	double dhstd_sum = 0;

	int acc_acc_cnt = 0;


	////------------------------------------------------------------------------
	////	
	////------------------------------------------------------------------------	
	FILE* fIMU = fopen(input_file_imu, "r");
	
	int read_cnt = 0;		// temporary

	pprawimu = (type_raw_imu **) new type_raw_imu *[MAX_STRUCTSIZE];

	for (int k = 0; k < MAX_STRUCTSIZE; ++k)
		pprawimu[k] = (type_raw_imu *) new type_raw_imu;

	read_cnt = 0;
	while (!feof(fIMU) && read_cnt < MAX_STRUCTSIZE)
	{
		int flag_read = read_IMU(pprawimu[read_cnt++], fIMU);
		if (flag_read == -1)
			break;
		if (flag_read == 0)
			read_cnt = read_cnt - 1;

	}

	end_line = read_cnt - 2;

	if (fIMU)	fclose(fIMU);

	TRACE("\n=========================================================\n");
	TRACE("NOTICE: read IMU data complete...");
	TRACE("\n=========================================================\n");
	str = " - read IMU data complete...";
	mc_pLB->AddString(str);
	mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);

	/////////////////// initialization ///////////////////
	num_calgn = 100.0; // 초반 측정치 안정화를 위한 측정치 평균 범위
	ini_imu_flag = 1;
	for (Sync_idx_imu = 0; Sync_idx_imu < num_calgn; Sync_idx_imu++)
	{
		for (i = 0; i < 3; i++)
		{
			axyzsum[i] = axyzsum[i] + pprawimu[Sync_idx_imu]->fb[i];
			rxyzsum[i] = rxyzsum[i] + pprawimu[Sync_idx_imu]->rbib[i];			
		}

		acc_buf[0][num_calgn - Sync_idx_imu - 1] = pprawimu[Sync_idx_imu]->fb[0];
		acc_buf[1][num_calgn - Sync_idx_imu - 1] = pprawimu[Sync_idx_imu]->fb[1];
	}

	for (j = 0; j < 3; j++)
	{
		avr.fb[j] = axyzsum[j] / num_calgn; // 평균치 획득
		avr.rbib[j] = rxyzsum[j] / num_calgn;
	}


	////------------------------------------------------------------------------
	////	
	////------------------------------------------------------------------------
	int flag_brm = 1;


	//// M002
	double ttgpst = 194874.0;
	double ttgpst_adder = 74.0;


	//// M003
	//double ttgpst = 195181.0;
	//double ttgpst_adder = 102.0;

	// M003
	//double ttgpst = 195559.0;
	//double ttgpst_adder = 102.0;

	//double ttgpst = .0;
	//double ttgpst_adder = .0;

	gps_read_cnt = 0;
	int ini_read;
	int ini_gps_f = 0;

	FILE *fp_inssol = fopen(output_file_inssol, "w");

	

	////------------------------------------------------------------------------
	////	
	////------------------------------------------------------------------------

	FILE *fp_input;
	if ((fp_input = fopen(input_file_gps, "r")) == NULL)
	{
		TRACE("Open file error!!\n");
		str = " - Open GPS file error...";
		mc_pLB->AddString(str);
		mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);
	}
	else
	{
		while (!feof(fp_input))
		{
			int flag_read = 0;

			char stmp[MAX_LINE * 2];

			fgets(stmp, sizeof(stmp), fp_input);

			char *tmp;
			tmp = strtok(stmp, "\t");		// gps time
			gps_sol.gps_time = atof(tmp);

			while (flag_read == 0)
			{
				flag_read = Parser(stmp, tmp, gps_sol);

				int proc_type = 0;

				if (Sync_idx_imu <= end_line)
				{
					double dIMUEPS = 0.001;

					if (pprawimu[Sync_idx_imu]->gps_time < ini_gpstime + 1)
					{
						while (1)
						{
							//printf("\b\t%f\t%f\n", pprawimu[Sync_idx_imu]->gps_time, ini_gpstime + 1);
							if (pprawimu[Sync_idx_imu]->gps_time >= ini_gpstime + 1)
								break;
							else
								Sync_idx_imu++;
						}
						TRACE("\n\t initial gpst and imut found ...\n");
						str = " - initial gpst and imut found...";
						mc_pLB->AddString(str);
						mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);

					}
					if (ini_imu_flag == 0)
					{
						double adder = 1.0;

						//while(pprawimu[Sync_idx_imu]->gps_time >= gps_sol.gps_time && pprawimu[Sync_idx_imu]->gps_time < gps_sol.gps_time+1)
						while (pprawimu[Sync_idx_imu]->gps_time >= gps_sol.gps_time && pprawimu[Sync_idx_imu]->gps_time < gps_sol.gps_time + adder)
						{

							////------------------------------------------------
							//// 정지상태 판단
							////------------------------------------------------

							// shift
							for (int ss = 20-1; ss > 0; --ss)
							{
								acc_buf[0][ss] = acc_buf[0][ss - 1];
								acc_buf[1][ss] = acc_buf[1][ss - 1];
								hacc[ss] = hacc[ss - 1];
							}

							// copy new one
							acc_buf[0][0] = pprawimu[Sync_idx_imu]->fb[0];
							acc_buf[1][0] = pprawimu[Sync_idx_imu]->fb[1];
							hacc[0] = sqrt(acc_buf[0][0] * acc_buf[0][0] + acc_buf[1][0] * acc_buf[1][0]);

							// update mean
							dhacc_sum = .0;
							for (int ss = 0; ss < 20; ++ss)
								dhacc_sum += hacc[ss];
							hacc_mean = dhacc_sum / 20.0;

							// update std
							dhstd_sum = .0;
							for (int ss = 0; ss < 20; ++ss)
							{
								dhstd_sum += ((hacc[ss] - hacc_mean)*(hacc[ss] - hacc_mean));
							}
							hacc_std = sqrt(dhstd_sum / (20.0 - 1.0));

							// update decision
							if (hacc_std <= (0.03 / (1.0 / IMU_SMPL_PERIOD)))
							{
								TRACE("\n # STOP \n");

								if (stop_sts == 0)	// previous = moving
								{
									for (i = 0; i < 3; i++)
									{
										axyzsum[i] = 0;// axyzsum[i] + pprawimu[Sync_idx_imu]->fb[i];
										rxyzsum[i] = 0;// rxyzsum[i] + pprawimu[Sync_idx_imu]->rbib[i];
									}
									acc_acc_cnt = 0;
								}

								stop_sts = 1;	// stop								
							}								
							else
							{
								TRACE("\n # MOVING \n");
								
								if (stop_sts == 1)	// previous = stop
								{
									if (acc_acc_cnt >= 100)
									{
										// course align again

										for (j = 0; j < 3; j++)
										{
											avr.fb[j] = axyzsum[j] / acc_acc_cnt; // 평균치 획득
											avr.rbib[j] = rxyzsum[j] / acc_acc_cnt;
										}

										course_align(&avr, &h, &m, &gps_sol);			// 초기 개략 정렬
									}
								}

								stop_sts = 0;	// moving								
							}
								

							stop_sts = 0;
							if (stop_sts == 1)	// stop
							{
								if (acc_acc_cnt < 500)
								{
									for (i = 0; i < 3; i++)
									{
										axyzsum[i] = axyzsum[i] + pprawimu[Sync_idx_imu]->fb[i];
										rxyzsum[i] = rxyzsum[i] + pprawimu[Sync_idx_imu]->rbib[i];
									}
									acc_acc_cnt += 1;
								}
								
							}
							else if (stop_sts == 0)	// moving
							{
								if ((gps_sol.gps_time >= ttgpst) && (gps_sol.gps_time < (ttgpst + ttgpst_adder)))	// 음영지모사
								{

									/////////////////// SDINS Algorithm ///////////////////
									pps.gps_time = pprawimu[Sync_idx_imu]->gps_time;
									avr.gps_time = pprawimu[Sync_idx_imu]->gps_time;

									//brm_avr_imu_pp(&h, &pprawimu[Sync_idx_imu], &avr, Sync_idx_imu, flag_brm); // 측정치 바이어스 제거

									do_hspd_sdins_in_rx_seq(&h, &m, &pps, &avr); // 자세계산
									// 자세계산 함수 내부에서 속도계산 함수(do_mspd_sdins_in_rx_seq_lvarm)를 호출함
									////////////////// SDINS Algorithm finish ///////////////////

									double tmpc, RPY[3], relpos[3];

									tmpc = 1 - pow(h.c[6], 2.0);
									tmpc = sqrt(tmpc);

									RPY[0] = atan2(h.c[7], h.c[8]);
									RPY[1] = atan2(-h.c[6], tmpc);
									RPY[2] = atan2(h.c[3], h.c[0])*r2d;

									if (RPY[2] < 0)
										RPY[2] = RPY[2] + 360;


									//fp_inssol = fopen(NAVOUTFILE, "a+");

									fprintf(fp_inssol, " %20.10f", pprawimu[Sync_idx_imu]->gps_time);
									fprintf(fp_inssol, " %20.10f", gps_sol.gps_time);

									fprintf(fp_inssol, " %20.10f", RPY[0] * r2d); // 3
									fprintf(fp_inssol, " %20.10f", RPY[1] * r2d);
									fprintf(fp_inssol, " %20.10f", RPY[2]);

									fprintf(fp_inssol, " %20.10f", m.lat*r2d); // 6
									fprintf(fp_inssol, " %20.10f", m.lon*r2d);
									fprintf(fp_inssol, " %20.10f", m.hgt);

									fprintf(fp_inssol, " %20.10f", gps_sol.lat); // 9
									fprintf(fp_inssol, " %20.10f", gps_sol.lon);
									fprintf(fp_inssol, " %20.10f", gps_sol.hgt);
									fprintf(fp_inssol, " %20.10f", gps_sol.hd);

									fprintf(fp_inssol, " %20.10f", gps_sol.vel_horz);	//13
									//fprintf(fp_inssol, "%20.10f", sqrt(m.v[0] * m.v[0] + m.v[1] * m.v[1]));

									/*fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[0]);	//14
									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[1]);
									fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[2]);

									fprintf(fp_inssol," %20.10f",gps_sol.GDOP);				//17
									fprintf(fp_inssol," %20.10f",gps_sol.PDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.HDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.TDOP);
									fprintf(fp_inssol," %20.10f",gps_sol.VDOP);

									fprintf(fp_inssol," %d",pps.m_up_type)*/;

									fprintf(fp_inssol, " 1 \n");

									//fclose(fp_inssol);

								}
								else
								{
									if (pprawimu[Sync_idx_imu]->gps_time == gps_sol.gps_time)
									{
										/////////////////// SDINS Algorithm ///////////////////
										pps.gps_time = pprawimu[Sync_idx_imu]->gps_time;
										avr.gps_time = pprawimu[Sync_idx_imu]->gps_time;

										//brm_avr_imu_pp(&h, &pprawimu[Sync_idx_imu], &avr, Sync_idx_imu, flag_brm); // 측정치 바이어스 제거

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

										RPY[0] = atan2(h.c[7], h.c[8]);
										RPY[1] = atan2(-h.c[6], tmpc);
										RPY[2] = atan2(h.c[3], h.c[0])*r2d;
										if (RPY[2] < 0)
											RPY[2] = RPY[2] + 360;


										if (0) {

										}
										else {
											//fp_inssol = fopen("ins_sol.txt", "a+");

											fprintf(fp_inssol, " %20.10f", pprawimu[Sync_idx_imu]->gps_time);	// 1
											fprintf(fp_inssol, " %20.10f", gps_sol.gps_time);					// 2

											fprintf(fp_inssol, " %20.10f", RPY[0] * r2d);						// 3
											fprintf(fp_inssol, " %20.10f", RPY[1] * r2d);						// 4
											fprintf(fp_inssol, " %20.10f", RPY[2]);								// 5

											fprintf(fp_inssol, " %20.10f", m.lat*r2d);							// 6
											fprintf(fp_inssol, " %20.10f", m.lon*r2d);							// 7
											fprintf(fp_inssol, " %20.10f", m.hgt);								// 8

											fprintf(fp_inssol, " %20.10f", gps_sol.lat);						// 9
											fprintf(fp_inssol, " %20.10f", gps_sol.lon);						// 10
											fprintf(fp_inssol, " %20.10f", gps_sol.hgt);						// 11
											fprintf(fp_inssol, " %20.10f", gps_sol.hd);							// 12

											fprintf(fp_inssol, " %20.10f", gps_sol.vel_horz);					//13

											/*fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[0]);	//14
											fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[1]);
											fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[2]);

											fprintf(fp_inssol," %20.10f",gps_sol.GDOP);				//17
											fprintf(fp_inssol," %20.10f",gps_sol.PDOP);
											fprintf(fp_inssol," %20.10f",gps_sol.HDOP);
											fprintf(fp_inssol," %20.10f",gps_sol.TDOP);
											fprintf(fp_inssol," %20.10f",gps_sol.VDOP);

											fprintf(fp_inssol," %d",pps.m_up_type)*/;

											fprintf(fp_inssol, " 0 \n");

											//fclose(fp_inssol);
										}


										/////////////////// GPS_INS Integration finish ///////////////////
									}
									else
									{
										/////////////////// SDINS Algorithm ///////////////////
										pps.gps_time = pprawimu[Sync_idx_imu]->gps_time;
										avr.gps_time = pprawimu[Sync_idx_imu]->gps_time;

										//brm_avr_imu(&h, &pprawimu[Sync_idx_imu], &avr,Sync_idx_imu, 0); // 측정치 바이어스 제거
										//brm_avr_imu_pp(&h, &pprawimu[Sync_idx_imu], &avr, Sync_idx_imu, flag_brm); // 측정치 바이어스 제거

										do_hspd_sdins_in_rx_seq(&h, &m, &pps, &avr); // 자세계산
										// 자세계산 함수 내부에서 속도계산 함수(do_mspd_sdins_in_rx_seq_lvarm)를 호출함
										////////////////// SDINS Algorithm finish ///////////////////

										double tmpc, RPY[3], relpos[3];

										tmpc = 1 - pow(h.c[6], 2.0);
										tmpc = sqrt(tmpc);

										RPY[0] = atan2(h.c[7], h.c[8]);
										RPY[1] = atan2(-h.c[6], tmpc);
										RPY[2] = atan2(h.c[3], h.c[0])*r2d;
										if (RPY[2] < 0)
											RPY[2] = RPY[2] + 360;

										if (0) {

										}
										else {
											//fp_inssol = fopen("ins_sol.txt", "a+");

											fprintf(fp_inssol, " %20.10f", pprawimu[Sync_idx_imu]->gps_time);
											fprintf(fp_inssol, " %20.10f", gps_sol.gps_time);

											fprintf(fp_inssol, " %20.10f", RPY[0] * r2d); // 3
											fprintf(fp_inssol, " %20.10f", RPY[1] * r2d);
											fprintf(fp_inssol, " %20.10f", RPY[2]);

											fprintf(fp_inssol, " %20.10f", m.lat*r2d); // 6
											fprintf(fp_inssol, " %20.10f", m.lon*r2d);
											fprintf(fp_inssol, " %20.10f", m.hgt);

											fprintf(fp_inssol, " %20.10f", gps_sol.lat); // 9
											fprintf(fp_inssol, " %20.10f", gps_sol.lon);
											fprintf(fp_inssol, " %20.10f", gps_sol.hgt);
											fprintf(fp_inssol, " %20.10f", gps_sol.hd);

											fprintf(fp_inssol, " %20.10f", gps_sol.vel_horz);	//13

											/*fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[0]);	//14
											fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[1]);
											fprintf(fp_inssol," %20.10f",pprawimu[Sync_idx_imu]->rbib[2]);

											fprintf(fp_inssol," %20.10f",gps_sol.GDOP);				//17
											fprintf(fp_inssol," %20.10f",gps_sol.PDOP);
											fprintf(fp_inssol," %20.10f",gps_sol.HDOP);
											fprintf(fp_inssol," %20.10f",gps_sol.TDOP);
											fprintf(fp_inssol," %20.10f",gps_sol.VDOP);

											fprintf(fp_inssol," %d",pps.m_up_type)*/;

											fprintf(fp_inssol, " 1 \n");

											//fclose(fp_inssol);
										}
									}
								}
							}
														

							Sync_idx_imu++;
						}
					}

					//if (ini_imu_flag && gps_sol.vel_horz > 1.0)
					//if(ini_imu_flag && gps_sol.vel_horz > 0.5)
					//if (ini_imu_flag && gps_sol.vel_horz > 0.02)
					//if (ini_imu_flag && gps_sol.vel_horz > 0.5)
					//if (ini_imu_flag && gps_sol.vel_horz > 0.25)
					//if (ini_imu_flag && gps_sol.vel_horz > 0.075)
					if (ini_imu_flag && gps_sol.vel_horz > 0.1)
						//if (ini_imu_flag && gps_sol.vel_horz > 0.2)
					{
						nav_type_init_tight(&avr, &h, &m, &pps);		// SDINS 관련 변수 초기화
						course_align(&avr, &h, &m, &gps_sol);			// 초기 개략 정렬


						str =  " # Course align result";
						mc_pLB->AddString(str);
						mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);

						str.Format("    - roll(deg) = %.10f\n", avr.ca_roll);
						mc_pLB->AddString(str);
						mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);
						str.Format("    - pitch(deg) = %.10f\n", avr.ca_pitch);
						mc_pLB->AddString(str);
						mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);
						str.Format("    - heading(deg) = %f\n", avr.ca_yaw);
						mc_pLB->AddString(str);
						mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);


						init_nav_by_lgps(gps_sol, m, h);				// GPS+INS 결합 관련 변수 초기화
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
		TRACE("Finished");
		str = " - post-processing finished...";
		mc_pLB->AddString(str);
		mc_pLB->SendMessage(WM_VSCROLL, SB_BOTTOM);
	}
	fclose(fp_inssol);


	////------------------------------------------------------------------------
	////	release memory
	////------------------------------------------------------------------------
	if (pprawimu)
	{
		for (int k = 0; k < MAX_STRUCTSIZE; ++k)
		{
			delete pprawimu[k];
		}
		delete[] pprawimu;
	}	

	return 0;
}



int CRTNetSolutionDlg::Parser(char* stmp, char *tmp, type_gps_sol& gps_sol)
{
	tmp = strtok(NULL, "\t");

	if (tmp != NULL)
	{
		//gps_sol.gps_time = atof(tmp);	// 													//2
		tmp = strtok(NULL, "\t");															//3
		tmp = strtok(NULL, "\t");															//4
		tmp = strtok(NULL, "\t");															//5
		tmp = strtok(NULL, "\t");															//6
		tmp = strtok(NULL, "\t");															//7
		tmp = strtok(NULL, "\t");															//8
		tmp = strtok(NULL, "\t");															//9

		tmp = strtok(NULL, "\t");		gps_sol.lat = atof(tmp);		// Lat (deg)		//10
		tmp = strtok(NULL, "\t");		gps_sol.lon = atof(tmp);		// Lon (deg)		//11
		tmp = strtok(NULL, "\t");		gps_sol.hgt = atof(tmp);		// Hgt (m)			//12

		tmp = strtok(NULL, "\t");															//13							
		tmp = strtok(NULL, "\t");															//14
		tmp = strtok(NULL, "\t");															//15

		tmp = strtok(NULL, "\t");		gps_sol.vel[0] = atof(tmp);							//16
		tmp = strtok(NULL, "\t");		gps_sol.vel[1] = atof(tmp);							//17
		tmp = strtok(NULL, "\t");		gps_sol.vel[2] = atof(tmp);							//18

		tmp = strtok(NULL, "\t");		gps_sol.vel_horz = atof(tmp);						//19
		tmp = strtok(NULL, "\t");		gps_sol.hd = atof(tmp);			// Heading(deg)		//20


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


int	CRTNetSolutionDlg::GetGPSTfrmUTC(int year, int month, int day, int hour, int minute, double second, double *gpstime, int &leapsec)
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
