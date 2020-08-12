#ifdef _WIN32
#pragma once
#define _CRT_SECURE_NO_WARNINGS   // to use scanf instead of scanf_s (scanf_s is not working in Linux).
#include <Windows.h>
#include <SDKDDKVer.h>
#include <tchar.h>
#include <string>
#include <conio.h>
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "imgcodecs.hpp"
#include <mutex>
#endif

#ifdef __linux__
#include "opencv2/opencv.hpp"
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <mutex>
#include <chrono>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <libudev.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <sys/ioctl.h>
enum uvc_req_code {
	UVC_RC_UNDEFINED = 0x00,
	UVC_SET_CUR = 0x01,
	UVC_GET_CUR = 0x81,
	UVC_GET_MIN = 0x82,
	UVC_GET_MAX = 0x83,
	UVC_GET_RES = 0x84,
	UVC_GET_LEN = 0x85,
	UVC_GET_INFO = 0x86,
	UVC_GET_DEF = 0x87
};

#define EXTENSION_UNIT_ID                   3

#define H264NODE_NAME "e-CAM22_USB: Record"
#define NON_H264NODE  "e-CAM22_USB: Preview"
#define HYPERYON_PID "c123"
#define H264_NODE_FMT "H264"

#define LOWORD(l) ((WORD)(l))
#define HIWORD(l) ((WORD)(((DWORD)(l) >> 16) & 0xFFFF))
#define LOBYTE(w) ((BYTE)(w))
#define HIBYTE(w) ((BYTE)(((WORD)(w) >> 8) & 0xFF))
#define WORD __uint16_t
#define DWORD __uint32_t
#define BYTE __uint8_t


bool set_xu_cmd(__u8 controlId, uint16_t setVal, int *desc);
bool get_xu_cmd(__u8 controlId, __u8 queryType, __u8 &outputValues, int *desc);
void *record_h264(void *arg);
void timestamp_string(char * myString);
bool stop_record;
bool record_menu();
#endif

#include <stdio.h>
#include <iostream>
#include <fstream>      // Added to Save buffer as .raw format
#include <thread>
#include <vector>

using namespace std;
using namespace cv;

#define EXIT				0
#define AUTO				1
#define MANUAL				2
#define AUTOANDMANUAL			3
#define	READFIRMWAREVERSION		0X40
#define BUFFERLENGTH			65
#define SDK_VERSION			"1.0.0"
#define UVCExtPROP    13

#ifdef _WIN32

#define PROPERTY			19

// eCAM22 Dual Stream APIs (eCAMFWExt.dll)
typedef BOOL(*ReadfirmwareversionECAM22_t) (UINT8, UINT8 *);
typedef BOOL(*GetRangeECAM22_t) (UINT8, UINT8 *, UINT8 *, UINT8 *);
typedef BOOL(*SetUVCEXTCntrlECAM22_t) (UINT8, UINT8);
typedef BOOL(*GetUVCEXTCntrlECAM22_t) (UINT8, UINT8 *);
typedef BOOL(*InitextensionunitECAM22_t) (TCHAR *);
typedef BOOL(*DeinitextensionunitECAM22_t) ();
typedef BOOL(*RestoreDefaultECAM22_t) ();

ReadfirmwareversionECAM22_t readfirmwareversion;
GetRangeECAM22_t GetVideoControlRangeECAM22;
SetUVCEXTCntrlECAM22_t SetVideoControlECAM22;
GetUVCEXTCntrlECAM22_t GetVideoControlRequestECAM22;
InitextensionunitECAM22_t initextensionunit;
DeinitextensionunitECAM22_t deinitextensionunit;
RestoreDefaultECAM22_t RestoreDefaultECAM22;

// H264 Recording APIs (MFTopologydll.dll)
typedef HRESULT(*MFInit_t) (TCHAR *);
typedef HRESULT(*MFDeInit_t) ();
typedef HRESULT(*MFStartRecording_t) (TCHAR *, int, int, int);
typedef HRESULT(*MFStopRecording_t) ();

MFInit_t MFInit;
MFDeInit_t MFDeInit;
MFStartRecording_t MFStartRecording;
MFStopRecording_t MFStopRecording;

int RecWidth, RecHeight, RecFps;
UINT8 CBRVal = 0, HDRValue = 0;

#define H264NODE_NAME								"e-CAM22_USB"
#define NON_H264NODE								"e-CAM22_USB-1"

#elif __linux__

__u8 CBRValue = 0;
__u8 HDRValue = 0;
#define PROPERTY			17

#endif

#define EXTENSION_UNIT_ID							3
#define ECAM22_XU_FW_VERSION						0x07
#define ECAM22_XU_NOISE_REDUCION					0x08
#define ECAM22_XU_HDR								0x03
#define ECAM22_XU_DEWARPING							0x0A
#define ECAM22_XU_EXPOSURE_ROI_MODE					0x0C
#define ECAM22_XU_EXPOSURE_ROI_MOUSE_CLICK			0x0D
#define ECAM22_XU_EXPOSURE_ROI_WINSIZE				0x0E

#define ECAM22_XU_BITRATE							0x02
#define ECAM22_XU_H264QUALITY						0x09
#define ECAM22_XU_QFACTOR							0x01
#define ECAM22_XU_CBR								0x11
#define ECAM22_XU_RESTORE_DEFAULT					0x06


struct hyperyon
{
	int o_index;
	string o_device_name;
	string o_vid;
	string o_pid;
	string o_device_path;
	double Zoom;
	double Tilt;
	double Pan;
	int fd;
}H264_node, MJPG_node;

//Variable Declarations
VideoCapture cap, cap_h264, *node_choice = NULL;
Mat Frame, Frame_h264;
char keyPressed = '\0', dilemma = 'y';
int devices = 0, formats = 0, width = 0, curWidth = 0, curHeight = 0, height = 0, fps = 0;
int minimum = 0, maximum = 0, defaultValue = 0, currentValue = 0, steppingDelta = 0, supportedMode = 0, currentMode = 0, value = 0;
vector< pair <int, String> > uvcProperty;
vector<pair <hyperyon, hyperyon> > cam_list;
vector < pair <uint8_t, String> > UVCExtension;
int node = -1;
int camId = -1;
String deviceName, vid, pid, devicePath, formatType;
bool bOpenHID = false, bCapture, bPreview, bSwitch, H264flag = false, bRecStarted = false;
mutex mu;
bool checkFormat = true;

static int *fd_choice = NULL;

int Hyp_Dev_Cnt = 0;
int addedNodes = 0;

#ifdef _WIN32

TCHAR *tDevicePath, *FileSavePath;
HINSTANCE hUVCInstLib, RecrdInstLib;
bool bDetach = false;
thread t;

#elif __linux__

pthread_t threadId, threadH264;

#endif


//Function Declarations
bool listDevices();
bool exploreCam();

void GetFirmware_version();
void choose_node();

bool bReadSet(int tid, bool bRead)
{
	std::lock_guard<std::mutex> guard(mu);

	if (tid == 1)
	{
		bCapture = bRead;
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}

	return bCapture;
}

bool bPreviewSet(int tid, bool bPrev)
{
	std::lock_guard<std::mutex> guard(mu);

	if (tid == 1)
	{
		bPreview = bPrev;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	return bPreview;
}

// To get the Format string from Fourcc value. by Murali.
String DwordToFourCC(double fcc)
{
	char buffer[4] = { "" };

	buffer[3] = HIBYTE(HIWORD(fcc));
	buffer[2] = LOBYTE(HIWORD(fcc));
	buffer[1] = HIBYTE(LOWORD(fcc));
	buffer[0] = LOBYTE(LOWORD(fcc));

	return buffer;
}

// To get weather the format is raw supported or not
bool IsRawSaveSupport()
{
	String Format = DwordToFourCC(cap.get(CV_CAP_PROP_FOURCC));

	if (Format.substr(0, 4) == "UYVY")
		return true;
	return false;
}

// To get the current set format and Resolution,when application launches.
bool getCurrentFormat() {

	curWidth = Frame.cols;
	curHeight = Frame.rows;

	return true;
}

// Write frame data into raw file
void SaveInRAW(uchar* Buffer, char* buf, int FrameSize)
{
	ostringstream filename;
	filename << buf;
	std::ofstream outfile(filename.str().c_str(), ios::out | ios::binary);
	outfile.write((char*)(Buffer), FrameSize);
	outfile.close();
	return;
}


void GetFirmware_version()
{

#ifdef _WIN32
	UINT8 outputValues[4];

	BOOL result = readfirmwareversion(ECAM22_XU_FW_VERSION, outputValues);
	if (!result)
	{
		cout << endl << "Reading Data from the UVC Extension is Failed" << endl;
		return;
	}

	cout << "\n\tCurrent Firmware Verion : ";
	cout << (int)outputValues[0] << "." << (int)outputValues[1] << "." << (int)outputValues[2] << "." << (int)outputValues[3] << endl;

#elif __linux__
	struct uvc_xu_control_query xquery;
	__u16 size = 0;
	__u8 outputValues[4];
	int ret = -1;
	memset(&xquery, 0, sizeof(xquery));

	// To allocate a sufficiently large buffer and set the buffer size to the correct value
	xquery.query = UVC_GET_LEN;
	xquery.size = 2;
	xquery.selector = ECAM22_XU_FW_VERSION;
	xquery.unit = EXTENSION_UNIT_ID;
	xquery.data = (__u8 *)&size;

	ret = ioctl(*fd_choice, UVCIOC_CTRL_QUERY, &xquery);
	if (ret<0)
	{
		perror("ioctl failed");
		return;
	}

	memset(&xquery, 0, sizeof(xquery));

	// get value from camera
	xquery.query = UVC_GET_CUR;
	xquery.size = size;
	xquery.selector = ECAM22_XU_FW_VERSION;
	xquery.unit = EXTENSION_UNIT_ID;
	xquery.data = (__u8 *)&outputValues;


	ret = ioctl(*fd_choice, UVCIOC_CTRL_QUERY, &xquery);
	if (ret<0)
	{
		perror("ioctl failed");
		return;
	}

	cout << "\n\tCurrent Firmware Verion : ";
	cout << (int)outputValues[0] << "." << (int)outputValues[1] << "." << (int)outputValues[2] << "." << (int)outputValues[3] << endl;

#endif
}


#ifdef _WIN32

//Video Record for Windows
//
void Recordvideo()
{

	bRecStarted = true;

	time_t t = time(0);
	char buf[MAX_PATH];
	struct tm tm;
	localtime_s(&tm, &t);
	HRESULT hr = E_FAIL;

	if (!cap_h264.isOpened())
	{
		if (!cap_h264.open(cam_list[camId - 1].second.o_index))
			cout << "Failed to Open " << cam_list[camId - 1].second.o_device_name << " Device" << endl;
	}

	RecWidth = (int)cap_h264.get(CV_CAP_PROP_FRAME_WIDTH);
	RecHeight = (int)cap_h264.get(CV_CAP_PROP_FRAME_HEIGHT);
	RecFps = (int)cap_h264.get(CV_CAP_PROP_FPS);

	// Zoom, Tilt, Pan Settings.
	cam_list[camId - 1].second.Zoom = cap_h264.get(CV_CAP_PROP_ZOOM);
	cam_list[camId - 1].second.Tilt = cap_h264.get(CV_CAP_PROP_TILT);
	cam_list[camId - 1].second.Pan = cap_h264.get(CV_CAP_PROP_PAN);

	cam_list[camId - 1].first.Zoom = cap.get(CV_CAP_PROP_ZOOM);
	cam_list[camId - 1].first.Tilt = cap.get(CV_CAP_PROP_TILT);
	cam_list[camId - 1].first.Pan = cap.get(CV_CAP_PROP_PAN);

	if ((cam_list[camId - 1].second.Zoom != cam_list[camId - 1].first.Zoom) || (cam_list[camId - 1].second.Tilt != cam_list[camId - 1].first.Tilt) || (cam_list[camId - 1].second.Pan != cam_list[camId - 1].first.Pan))
	{

		cout << "\n\tNOTE: Preview & Record node Settings are different." << endl;

		while (true)
		{
			cout << endl << "Enter y/Y to Apply or n/N to do not Apply: " << '\t';
			scanf("%c", &dilemma);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
			if (dilemma == 'y' || dilemma == 'Y')
			{
				if (cap_h264.isOpened())
				{
					if (!cap_h264.set(CV_CAP_PROP_ZOOM, cam_list[camId - 1].first.Zoom))
						cout << "Failed to set Zoom Value" << endl;
					if (!cap_h264.set(CV_CAP_PROP_TILT, cam_list[camId - 1].first.Tilt))
						cout << "Failed to set Tilt Value" << endl;
					if (!cap_h264.set(CV_CAP_PROP_PAN, cam_list[camId - 1].first.Pan))
						cout << "Failed to set Pan Value" << endl;
				}
				else
					cout << "Device is not Opned" << endl;
				break;
			}
			else if (dilemma == 'n' || dilemma == 'N')
			{
				break;
			}
		}
	}


	cap_h264.release();

	sprintf_s(buf, "Hyperyon_Cam_%dx%d_%d%d%d_%d%d%d.mp4", RecWidth, RecHeight, tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900, tm.tm_hour, tm.tm_min, tm.tm_sec);

	if (!cap_h264.isOpened())
	{
		tDevicePath = new TCHAR[H264_node.o_device_path.size() + 1];
		copy(H264_node.o_device_path.begin(), H264_node.o_device_path.end(), tDevicePath);

		hr = MFInit(tDevicePath);
		if (FAILED(hr))
		{
			cout << "MFInit Failed..." << endl;
		}

		FileSavePath = new TCHAR[MAX_PATH];
		mbstowcs(FileSavePath, buf, MAX_PATH);

		hr = MFStartRecording(FileSavePath, RecWidth, RecHeight, RecFps);
		if (FAILED(hr))
		{
			cout << "MFInit Failed..." << endl;
		}
		else
			cout << "Record Started..." << endl;
	}
	else
		cout << "device is busy" << endl;
}

//Preview Window for Windows
//
void stream()
{
	bCapture = true;
	while (true)
	{
		while (bPreviewSet(2, true))
		{
			if (bReadSet(2, true))
			{
				cap >> Frame;
			}
			if (!Frame.empty())
			{
				if (checkFormat)
				{  // Call getCurrentFormat API,where we will get the initial set format and Resolution.
					getCurrentFormat();
					checkFormat = false;  // Make it false,as it is used only once,just to get the initial set values.
				}
				namedWindow("Hyperyon_Cam", WINDOW_AUTOSIZE);
				imshow("Hyperyon_Cam", Frame);
			}
			keyPressed = waitKey(10);
			while (bSwitch)
			{
				bSwitch = false;
				destroyAllWindows();
			}
		}
	}
}

#elif __linux__

bool closeHID()
{

	if (cam_list[camId - 1].second.fd > 0)
		close(cam_list[camId - 1].second.fd);
	if (cam_list[camId - 1].first.fd >0)
		close(cam_list[camId - 1].first.fd);
	return true;
}

//Preview Window for Linux

void *preview(void *arg)
{
	bCapture = true;
	while (true)
	{
		while (bPreviewSet(2, true))
		{
			if (bReadSet(2, true))
			{
				cap >> Frame;
			}

			if (!Frame.empty())
			{
				if (checkFormat) {  // Call getCurrentFormat API,where we will get the initial set format and Resolution.
					getCurrentFormat();
					checkFormat = false;  // Make it false,as it is used only once,just to get the initial set values.
				}
				else
				{
					namedWindow("Hyperyon_Cam", WINDOW_AUTOSIZE);
					imshow("Hyperyon_Cam", Frame);
				}
			}

			keyPressed = waitKey(5);

			while (bSwitch)
			{
				bSwitch = false;
				destroyAllWindows();
			}
		}
	}
}

#endif

int main()
{
	//Basic Introduction about the Application
	cout << endl << "e-con's Sample OpenCV Hyperyon_Cam Application for Hyperyon " << endl;
	//    cout << endl << "Demonstrates the working of e-con's Custom Format cameras with the modified libraries of OpenCV" << endl;
	cout << endl << "\t" << "OpenCV Hyperyon_Cam SDK-Version = " << SDK_VERSION << "\n\n";

#ifdef _WIN32

	hUVCInstLib = LoadLibrary(L"eCAMFWExt.dll");
	if (hUVCInstLib == NULL)
	{
		cout << "The eCAMFWExt.dll is not loaded properly" << endl;
	}

	if (hUVCInstLib) {
		readfirmwareversion = (ReadfirmwareversionECAM22_t)GetProcAddress(hUVCInstLib, "GetFirmwareVersionECAM22");
		initextensionunit = (InitextensionunitECAM22_t)GetProcAddress(hUVCInstLib, "InitExtensionUnitECAM22");
		deinitextensionunit = (DeinitextensionunitECAM22_t)GetProcAddress(hUVCInstLib, "DeinitExtUnit");
		RestoreDefaultECAM22 = (RestoreDefaultECAM22_t)GetProcAddress(hUVCInstLib, "RestoreDefaultECAM22");

		GetVideoControlRangeECAM22 = (GetRangeECAM22_t)GetProcAddress(hUVCInstLib, "GetVideoControlRangeECAM22");
		SetVideoControlECAM22 = (SetUVCEXTCntrlECAM22_t)GetProcAddress(hUVCInstLib, "SetVideoControlECAM22");
		GetVideoControlRequestECAM22 = (GetUVCEXTCntrlECAM22_t)GetProcAddress(hUVCInstLib, "GetVideoControlRequestECAM22");
	}

	RecrdInstLib = LoadLibrary(L"MFTopologydll.dll");
	if (RecrdInstLib == NULL)
	{
		cout << "The MFTopologydll.dll is not loaded properly" << endl;
	}

	if (RecrdInstLib)
	{
		MFInit = (MFInit_t)GetProcAddress(RecrdInstLib, "MFInit");
		MFDeInit = (MFDeInit_t)GetProcAddress(RecrdInstLib, "MFDeInit");
		MFStartRecording = (MFStartRecording_t)GetProcAddress(RecrdInstLib, "MFStartRecording");
		MFStopRecording = (MFStopRecording_t)GetProcAddress(RecrdInstLib, "MFStopRecording");
	}

#endif

	//Open a Camera Device
	if (!(listDevices()))
	{
		cout << endl << "List Devices Information Failed" << endl;
		cout << endl << '\t' << "Press Any key to exit the Application: " << '\t';
#ifdef _WIN32
		_getch();
#endif
		return 0;
	}

#ifdef _WIN32
	t = thread(&stream);
#elif __linux__
	pthread_create(&threadId, NULL, preview, NULL);
#endif

	if (!(exploreCam()))
	{
		cout << endl << "Camera Exploration Failed" << endl;
		return 0;
	}

#ifdef _WIN32
	t.detach();
#endif

	if (cap.isOpened())
		cap.release();
	if (cap_h264.isOpened())
		cap_h264.release();

	return 0;
}


//Listing the Devices

bool listDevices()
{
	bool hyperyon_flag = false;
	Hyp_Dev_Cnt = 0;
	camId = -1;
	//List total Number of Devices
#ifdef __linux__
	bSwitch = true;
#endif
	if (!(cap.getDevices(devices)))
	{
		cout << endl << "Get total number of devices Failed" << endl;
		return false;
	}

	if (devices < 0)
	{
		cout << endl << "No Camera Devices Connected to the port" << endl;
		return false;
	}

	cout << endl << "Camera Devices Connected to the PC Port : " << endl << endl;
	cout << '\t' << "0 - Exit" << endl;

	//List the Camera Names
	for (int eachDevice = 0; eachDevice < devices; eachDevice++)
	{
		if (!(cap.getDeviceInfo(eachDevice, deviceName, vid, pid, devicePath)))
		{
			cout << endl << "Device " << eachDevice << " Information couldn't be Retrieved" << endl;
		}
#ifdef  __linux__
		if (pid == HYPERYON_PID)
		{
			hyperyon_flag = true;
			cap.open(devicePath);
			cap.getFormatType(0, formatType, width, height, fps);
			cap.release();
			if (formatType == H264_NODE_FMT)
			{
				H264_node.o_index = eachDevice;
				H264_node.o_device_name = H264NODE_NAME;
				H264_node.o_vid = vid;
				H264_node.o_pid = pid;
				H264_node.o_device_path = devicePath;
				addedNodes++;
			}
			else
			{
				MJPG_node.o_index = eachDevice;
				MJPG_node.o_device_name = NON_H264NODE;
				MJPG_node.o_vid = vid;
				MJPG_node.o_pid = pid;
				MJPG_node.o_device_path = devicePath;
				addedNodes++;
			}
		}
		if (addedNodes == 2)
		{
			Hyp_Dev_Cnt++;
			cam_list.push_back(make_pair(H264_node, MJPG_node));
			cout << '\t' << Hyp_Dev_Cnt << " . " << MJPG_node.o_device_name << endl;
			addedNodes = 0;
		}

#elif _WIN32

		if (deviceName == NON_H264NODE)
		{
			MJPG_node.o_index = eachDevice;
			MJPG_node.o_device_name = deviceName;
			MJPG_node.o_vid = vid;
			MJPG_node.o_pid = pid;
			MJPG_node.o_device_path = devicePath;

			hyperyon_flag = true;
			addedNodes++;

			if (addedNodes == 2)
			{
				cam_list.push_back(make_pair(MJPG_node, H264_node));

				cout << '\t' << Hyp_Dev_Cnt + 1 << " . " << H264NODE_NAME << endl;
				addedNodes = 0;
				Hyp_Dev_Cnt++;
			}
		}

		if (deviceName == H264NODE_NAME)
		{
			H264_node.o_index = eachDevice;
			H264_node.o_device_name = deviceName;
			H264_node.o_vid = vid;
			H264_node.o_pid = pid;
			H264_node.o_device_path = devicePath;

			addedNodes++;

			if (addedNodes == 2)
			{
				cam_list.push_back(make_pair(MJPG_node, H264_node));

				cout << '\t' << Hyp_Dev_Cnt + 1 << " . " << H264NODE_NAME << endl;
				addedNodes = 0;
				Hyp_Dev_Cnt++;
			}
		}
#endif

	}
	while ((camId < 0) || (camId > Hyp_Dev_Cnt))
	{
		printf("\n Pick a Camera Device to Explore : \t");
		scanf("%d", &camId);
		while (getchar() != '\n' && getchar() != EOF)
		{
		}
	}
	fflush(stdin);
	if (!hyperyon_flag)
	{
		cout << "Hypeyon not detected! exiting Application..";
		exit(0);
	}
	switch (camId)
	{
	case EXIT:
#ifdef _WIN32

		if (deinitextensionunit())
			if (bDetach)
			{
				t.detach();
			}
		MFDeInit();
		bSwitch = true;
		if (cap.isOpened())
			cap.release();

#elif __linux__

		bPreviewSet(1, false);
		if (closeHID())
			destroyAllWindows();

#endif

		exit(0);
		break;

	default:
		checkFormat = true;
		bPreviewSet(1, false);
#ifdef _WIN32
		cap.getDeviceInfo(cam_list[camId - 1].first.o_index, deviceName, vid, pid, devicePath);
#elif __linux__
		cap.getDeviceInfo((camId - 1), deviceName, vid, pid, devicePath);
#endif
		if (cap.isOpened())
			cap.release();


#ifdef _WIN32
		if (cap.open(cam_list[camId - 1].first.o_index))
#elif __linux__
		if (cap.open(cam_list[camId - 1].second.o_index))
#endif
		{
			if (!cap.isOpened())
			{
				cout << endl << "\t Camera Device not Initialised Successfully \n\n Press any Key to exit the application\n";
				return 0;
			}
		}

#ifdef _WIN32

		tDevicePath = new TCHAR[devicePath.size() + 1];
		copy(devicePath.begin(), devicePath.end(), tDevicePath);
		bOpenHID = initextensionunit(tDevicePath);

#elif __linux__
		if (cam_list[camId - 1].second.fd > 0)
			closeHID();
		std::string str = String(cam_list[camId - 1].second.o_device_path);
		const char * path = str.c_str();
		cam_list[camId - 1].second.fd = ::open(path, O_RDWR | O_NONBLOCK, 0);
		if (cam_list[camId - 1].second.fd < 0)
			bOpenHID = false;
		else
			bOpenHID = true;

#endif
		bSwitch = false;
		bPreviewSet(1, true);
		break;
	}
#ifdef _WIN32
	bDetach = true;
#endif
	return true;
}


//Configuring Camera Format/Resolution
bool configFormats(VideoCapture *node)
{
	while ((dilemma == 'y') || (dilemma == 'Y'))
	{
		int index = -1;

		if (!(node->getFormats(formats)))
		{
			cout << endl << "Get Total number of Formats Failed" << endl;
			perror("getFormats");
			return false;
		}

		cout << endl << "Total Number of Formats Supported by the Camera:  " << '\t' << formats << endl;

		cout << "\n\t0 - Exit" << endl;
		cout << "\t1 - Back" << endl;
		cout << "\t2 - Main Menu" << endl;
		int option = 3;

		for (int formatList = 0; formatList < formats; formatList++)
		{
			if (!(node->getFormatType(formatList, formatType, width, height, fps)))
			{
				cout << endl << "Camera Get Format Type Failed" << endl;
				return false;
			}

			cout << '\t' << option << " . " << "FormatType: " << formatType << " Width: " << width << " Height: " << height << " Fps: " << fps << endl;
			option++;
		}

		while ((index < 0) || (index >= option))
		{
			printf("\nPick a choice to set a Particular Preview Format: \t");
			scanf("%d", &index);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
		}

		switch (index)
		{
		case EXIT:
#ifdef _WIN32

			if (deinitextensionunit())
			{
				t.detach();
			}
			MFDeInit();
			bSwitch = true;
			if (cap.isOpened())
				cap.release();

#elif __linux__

			bPreviewSet(1, false);
			if (closeHID())
				destroyAllWindows();

#endif

			exit(0);

		case 1:
		case 2:
			exploreCam();
			break;

		default:
			bSwitch = true;
			bPreviewSet(1, false);
			bReadSet(1, false);

			index = index - 3;

			if (index == -1)
			{
				cout << endl << "Invalid index value to configure the formats" << endl;
				return false;
			}
			if (!(node->setFormatType(index)))
			{
				cout << endl << "Camera Set Format Type Failed" << endl;
				return false;
			}
			if (node->getFormatType(index, formatType, width, height, fps))
				cout << "\n\t **** The Current set Format type = " << formatType << " Width = " << width << " Height = " << height << " FPS = " << fps << " ****\n\n";

			bReadSet(1, true);
			bPreviewSet(1, true);

			curWidth = width;
			curHeight = height;

			formatType = '\0';
			width = height = fps = 0;
			break;
		}
	}
	return true;
}

//Set Video Property
bool setVidProp(int Property, string PropStr, VideoCapture *node)
{
	while ((dilemma == 'y') || (dilemma == 'Y'))
	{
		int mode = 0;

		if (!(node->get(Property, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue)))
		{
			cout << endl << PropStr << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
			return false;
		}

		cout << endl << "Camera " << PropStr << " Values:: " << endl;

		if (supportedMode != AUTO)
		{
			cout << '\t' << "Minimum Value: " << minimum << endl;
			cout << '\t' << "Maximum Value: " << maximum << endl;
			cout << '\t' << "SteppingDelta: " << steppingDelta << endl;    //incrementing scale Value between min and max value
			cout << '\t' << "Default Value: " << defaultValue << endl;
		}

		((supportedMode < AUTOANDMANUAL) ? ((supportedMode == AUTO) ? (cout << '\t' << "Supported Mode: Auto" << endl) : (cout << '\t' << "Supported Mode: Manual" << endl)) : (cout << '\t' << "Supported Mode: Auto/Manual" << endl));

		if (currentMode == MANUAL)
			cout << '\t' << "Current Value: " << currentValue << endl;
		((currentMode == AUTO) ? (cout << '\t' << "Current Mode: Auto" << endl << endl) : (cout << '\t' << "Current Mode: Manual" << endl << endl));

		switch ((long)supportedMode)
		{
		case AUTO:
			cout << "Only Auto " << PropStr << " is Supported by the Camera, User can't set the Mode" << endl << endl;
			mode = AUTO;
			break;

		case MANUAL:
			cout << "Only Manual " << PropStr << " is Supported by the Camera, User can't set the Mode " << endl << endl;
			mode = MANUAL;
			break;

		case AUTOANDMANUAL:
			while ((mode <= 0) || (mode > 2))
			{
				printf("\n Enter a Valid mode to get selected: 1. Auto 2. Manual \n");
				scanf("%d", &mode);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
			}
			break;
		}

		while (mode == MANUAL)
		{
			cout << endl << "Enter a Valid value to Set " << PropStr << " : " << '\t';
			scanf("%d", &value);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}

			if ((value >= minimum) && (value <= maximum) && (((value) % (steppingDelta)) == 0))
			{
				break;
			}
			cout << endl << "Certain Conditions Not met; Please ";
		}

		if (!(node->set(Property, value, mode)))
		{
			cout << endl << "Camera " << PropStr << " is Not Set" << endl;
			return false;
		}

		((mode == AUTO) ? (cout << endl << "Auto " << PropStr << " Mode is Set" << endl) : (cout << endl << "Manual " << PropStr << " Mode is set with the Value : " << value << endl));

		cout << endl << "Camera " << PropStr << " Exploration: " << '\n';
		while (true)
		{
			cout << endl << "Enter y/Y to Continue or n/N to dis-Continue: " << '\t';
			scanf("%c", &dilemma);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
			if (dilemma == 'y' || dilemma == 'Y' || dilemma == 'n' || dilemma == 'N')
				break;
		}
	}

	return true;
}


//Configuring UVC Settings

bool configUVCSettings(VideoCapture *node)
{
#ifdef _WIN32

	int vid[PROPERTY] = { EXIT, 1, 2, CV_CAP_PROP_BRIGHTNESS, CV_CAP_PROP_CONTRAST, CV_CAP_PROP_SATURATION, CV_CAP_PROP_HUE, CV_CAP_PROP_GAIN, CV_CAP_PROP_EXPOSURE, CV_CAP_PROP_WHITE_BALANCE_BLUE_U, CV_CAP_PROP_SHARPNESS, CV_CAP_PROP_GAMMA, CV_CAP_PROP_ZOOM, CV_CAP_PROP_FOCUS, CV_CAP_PROP_BACKLIGHT, CV_CAP_PROP_PAN, CV_CAP_PROP_TILT, CV_CAP_PROP_ROLL, CV_CAP_PROP_IRIS };

	string vidStr[PROPERTY] = { "Exit", "Back", "Main Menu", "Brightness", "Contrast", "Saturation", "Hue", "Gain", "Exposure", "White Balance", "Sharpness", "Gamma", "Zoom", "Focus", "Backlight", "Pan", "Tilt", "Roll", "Iris" };

#elif __linux__

	int vid[PROPERTY] = { EXIT, 1, 2, CV_CAP_PROP_BRIGHTNESS, CV_CAP_PROP_CONTRAST, CV_CAP_PROP_SATURATION, CV_CAP_PROP_HUE, CV_CAP_PROP_GAIN, CV_CAP_PROP_EXPOSURE, CV_CAP_PROP_WHITE_BALANCE_BLUE_U, CV_CAP_PROP_SHARPNESS, CV_CAP_PROP_GAMMA, CV_CAP_PROP_ZOOM, CV_CAP_PROP_FOCUS, CV_CAP_PROP_BACKLIGHT, CV_CAP_PROP_PAN, CV_CAP_PROP_TILT };
	string vidStr[PROPERTY] = { "Exit", "Back", "Main Menu", "Brightness", "Contrast", "Saturation", "Hue", "Gain", "Exposure", "White Balance", "Sharpness", "Gamma", "Zoom", "Focus", "Backlight", "Pan", "Tilt" };

#endif

	while (true)
	{
		int choice = -1, settings = 0;

		for (int eachVideoSetting = 0; eachVideoSetting < PROPERTY; eachVideoSetting++)
		{
			if (!((eachVideoSetting >= 0) && (eachVideoSetting < 3)))
			{
				//Checks whether the Specific UVC Property is supported or not
				if (node->get(vid[eachVideoSetting], minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue))
				{
					cout << '\t' << settings << " - " << vidStr[eachVideoSetting] << endl;
					uvcProperty.push_back(make_pair(vid[eachVideoSetting], vidStr[eachVideoSetting]));
					settings++;
				}
				continue;
			}
			uvcProperty.push_back(make_pair(vid[eachVideoSetting], vidStr[eachVideoSetting]));
			cout << '\t' << settings << " - " << vidStr[eachVideoSetting] << endl;
			settings++;
		}

		while ((choice < 0) || (choice >= settings))
		{
			printf("\n Pick a Choice to Configure UVC Settings : \t");
			scanf("%d", &choice);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
		}

		dilemma = 'y';

		switch (uvcProperty[choice].first)
		{
		case EXIT:
#ifdef _WIN32

			if (deinitextensionunit())
			{
				t.detach();
			}
			MFDeInit();
			bSwitch = true;
			if (cap.isOpened())
				cap.release();

#elif __linux__

			bPreviewSet(1, false);
			if (closeHID())
				destroyAllWindows();

#endif

			exit(0);

		case 1:
		case 2:
			if (!(exploreCam()))
			{
				cout << endl << "Camera Exploration Failed" << endl;
				return false;
			}
			cout << endl << "Camera Exploration is done" << endl;
			break;

		case CV_CAP_PROP_BRIGHTNESS:
		case CV_CAP_PROP_CONTRAST:
		case CV_CAP_PROP_HUE:
		case CV_CAP_PROP_SATURATION:
		case CV_CAP_PROP_SHARPNESS:
		case CV_CAP_PROP_GAMMA:
		case CV_CAP_PROP_WHITE_BALANCE_BLUE_U:
		case CV_CAP_PROP_BACKLIGHT:
		case CV_CAP_PROP_GAIN:
		case CV_CAP_PROP_PAN:
		case CV_CAP_PROP_TILT:
		case CV_CAP_PROP_ROLL:
		case CV_CAP_PROP_ZOOM:
		case CV_CAP_PROP_EXPOSURE:
		case CV_CAP_PROP_IRIS:
		case CV_CAP_PROP_FOCUS:
			if (!(setVidProp(uvcProperty[choice].first, uvcProperty[choice].second, node)))
			{
				cout << endl << "Set Video Property Failed" << endl;
				return false;
			}
			cout << endl << "Camera " << uvcProperty[choice].second << " is Modified" << endl;
			break;
		}
	}

	return true;
}


//Capture Still Images
bool captureStill()
{
	int num;
	char buf[240], buf1[240];
	memset(buf, 0, 240);
	memset(buf1, 0, 240);
	time_t t = time(0);
	Mat stillFrame;
	bool IsRAWSelected = false;

	if (IsRawSaveSupport())
	{
		int choice = -1;
		cout << endl << '\t' << "0 - Back" << endl;
		cout << '\t' << "1 - RAW Format" << endl;
		cout << '\t' << "2 - RGB Format" << endl;

		while ((choice < 0) || (choice > 2))
		{
			printf("\n Pick a Relevant Choice to Configure Particular Camera Properties: \t");
			scanf("%d", &choice);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
		}
		switch (choice)
		{
		case 0:
			cout << "\n\tStill capture Skipped.." << endl;
			return true;
			break;
		case 1:
			IsRAWSelected = true;
			break;
		case 2:
			IsRAWSelected = false;
			break;
		}
	}


#ifdef _WIN32
	struct tm tm;
	localtime_s(&tm, &t);

	if (IsRAWSelected)
	{
		bPreviewSet(1, false);
		if (!cap.set(CV_CAP_PROP_CONVERT_RGB, false))
			cout << "\tCV_CAP_PROP_CONVERT_RGB is Disable Failed" << endl;

		num = sprintf_s(buf, "Hyperyon_Cam_%dx%d_%d%d%d_%d%d%d.raw", curWidth, curHeight, tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900, tm.tm_hour, tm.tm_min, tm.tm_sec);
	}
	else
		num = sprintf_s(buf, "Hyperyon_Cam_%dx%d_%d%d%d_%d%d%d.jpeg", curWidth, curHeight, tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900, tm.tm_hour, tm.tm_min, tm.tm_sec);

#elif __linux__

	struct tm *tm;
	tm = localtime(&t);

	char cwd[256];
	getcwd(cwd, sizeof(cwd));

	if (IsRAWSelected)
	{
		bPreviewSet(1, false);
		if (cap.set(CV_CAP_PROP_CONVERT_RGB, false))
			cout << "\tCV_CAP_PROP_CONVERT_RGB is Disabled" << endl;

		num = sprintf(buf, "OpenCVCam_%dx%d_%d%d%d_%d%d%d.raw", curWidth, curHeight, tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900, tm->tm_hour, tm->tm_min, tm->tm_sec);
	}
	else
		sprintf(buf, "%s/OpenCVCam_%dx%d_%d%d%d_%d%d%d.jpeg", cwd, curWidth, curHeight, tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900, tm->tm_hour, tm->tm_min, tm->tm_sec);


#endif

	if (cap.read(stillFrame))
	{
		if (!stillFrame.empty())
		{
			if (IsRAWSelected)
			{
				SaveInRAW(stillFrame.data, buf, (stillFrame.cols * stillFrame.rows * 2));
				if (!cap.set(CV_CAP_PROP_CONVERT_RGB, true))
					cout << "\tCV_CAP_PROP_CONVERT_RGB is Enable Failed" << endl;
				bPreviewSet(1, true);
			}
			else
			{
				imwrite(buf, stillFrame);
			}
			cout << endl << '\t' << buf << " image is saved " << endl << endl;
		}
	}

	memset(buf, 0, 240);
	memset(buf1, 0, 240);

	cout << endl << "Still Capture is Done" << endl;
	return true;
}

//HID Settings
//
bool hidProp(int *desc)
{
	uint8_t ctrlId[UVCExtPROP] = { EXIT,1,2,ECAM22_XU_RESTORE_DEFAULT,ECAM22_XU_FW_VERSION,ECAM22_XU_QFACTOR,ECAM22_XU_BITRATE,ECAM22_XU_H264QUALITY,ECAM22_XU_CBR,ECAM22_XU_NOISE_REDUCION,ECAM22_XU_DEWARPING,ECAM22_XU_EXPOSURE_ROI_MODE,ECAM22_XU_HDR };
	string vidCntrlStr[UVCExtPROP] = { "Exit", "Back", "Main Menu","Restore Default","GetFirmware Version Number", "MJPG Qfactor", "H264 Bitrate", "H264 Quality", "H264 CBR mode", "Denoise", "Dewarp", "ROI Auto Exposure Mode", "HDR" };
	UVCExtension.clear();

	while (true)
	{
		int choice = -1, settings = 0;

#ifdef __linux__
		if (node == 1)
			if (!get_xu_cmd(ECAM22_XU_CBR, UVC_GET_CUR, CBRValue, desc))
				cout << "\t H264 CBR Value failes to get" << endl;
		if (!get_xu_cmd(ECAM22_XU_HDR, UVC_GET_CUR, HDRValue, desc))
			cout << "\t ROI Mode failes to get" << endl;

#elif _WIN32
		if (node == 2)
			if (!GetVideoControlRequestECAM22(ECAM22_XU_CBR, &CBRVal))
				cout << "\t H264 CBR Value failes to get" << endl;

		if (!(GetVideoControlRequestECAM22(ECAM22_XU_HDR, &HDRValue)))
			cout << "\t ROI Mode failes to get" << endl;


#endif
		UVCExtension.clear();

		for (int eachVideoSetting = 0; eachVideoSetting < UVCExtPROP; eachVideoSetting++)
		{
			if (!((eachVideoSetting >= 0) && (eachVideoSetting < 4)))
			{
#ifdef __linux__
				if (HDRValue == 1)
					if (eachVideoSetting == 11)
						continue;
				if (node == 1)
					if (CBRValue == 1)
						if (eachVideoSetting == 7)
							continue;
				if (node == 1)
					if (eachVideoSetting == 5)
						continue;

				if (node == 2)
					if (eachVideoSetting == 6 || eachVideoSetting == 7 || eachVideoSetting == 8)
						continue;
#elif _WIN32

				if (node == 2)
					if (CBRVal == 1) // For H264 node, to disable the H264 Quality when the CBR Mode is ON(1).
						if (eachVideoSetting == 7)
							continue;

				if (HDRValue == 1) // To disable the ROI mode when the HDR  is HDR 2x mode.
					if (eachVideoSetting == 11)
						continue;

				if (node == 2) // Disabling the MJPG Q-Factor for H264 node
					if (eachVideoSetting == 5)
						continue;

				if (node == 1) // Disabling the H264 controls for MJPG node
					if (eachVideoSetting == 6 || eachVideoSetting == 7 || eachVideoSetting == 8)
						continue;
#endif
				cout << '\t' << settings << " - " << vidCntrlStr[eachVideoSetting] << endl;
				UVCExtension.push_back(make_pair(ctrlId[eachVideoSetting], vidCntrlStr[eachVideoSetting]));
				settings++;
				continue;
			}
			UVCExtension.push_back(make_pair(ctrlId[eachVideoSetting], vidCntrlStr[eachVideoSetting]));
			cout << '\t' << settings << " - " << vidCntrlStr[eachVideoSetting] << endl;
			settings++;
		}
		while ((choice < 0) || (choice > settings))
		{
			printf("\n Pick a Relevant Choice to Configure Particular Camera Properties: \t");
			scanf("%d", &choice);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
		}

#ifdef _WIN32
		UINT8 uCtrlMinValue, uCtrlMAxValue, uCtrlStepValue, uCtrlCurrentValue;;
#elif __linux__
		__u8 outputValues, min, max;
		__u16 set_value;
#endif

	REPEAT:
		switch (choice)
		{
		case EXIT:
#ifdef _WIN32

			if (deinitextensionunit())
			{
				t.detach();
			}
			MFDeInit();
			bSwitch = true;
			if (cap.isOpened())
				cap.release();

#elif __linux__

			bPreviewSet(1, false);
			if (closeHID())
				destroyAllWindows();

#endif

			exit(0);
		case 1:
		case 2:
			if (!(exploreCam()))
				cout << "Camera Exploration is Failed" << endl;
			break;


		default:
			switch (UVCExtension[choice].first)
			{
			case ECAM22_XU_FW_VERSION:
				GetFirmware_version();
				break;
			case ECAM22_XU_RESTORE_DEFAULT:
#ifdef _WIN32
				if (!RestoreDefaultECAM22())
					cout << "Restore Default Failed" << endl;
				else
					cout << endl << "\tAll Extension controls are set to Default" << endl;
#elif __linux__
				if (!set_xu_cmd(ECAM22_XU_RESTORE_DEFAULT, 0x01, desc))
					cout << "Restore Default Failed" << endl;
				else
					cout << endl << "\tAll Extension controls are set to Default" << endl;

#endif
				break;
			case ECAM22_XU_QFACTOR:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << endl << "Camera " << UVCExtension[choice].second << " Values:: " << endl;

				cout << '\t' << "Minimum Value: " << (int)uCtrlMinValue << endl;
				cout << '\t' << "Maximum Value: " << (int)uCtrlMAxValue << endl;
				cout << '\t' << "SteppingDelta: " << (int)uCtrlStepValue << endl;    //incrementing scale Value between min and max value
				cout << '\t' << "Current Value: " << (int)uCtrlCurrentValue << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is set with the Value : " << value << endl;


#elif __linux__
				cout << endl << "Camera " << UVCExtension[choice].second << " Values:: " << endl;

				get_xu_cmd(ECAM22_XU_QFACTOR, UVC_GET_MIN, outputValues, desc);
				cout << "Minimum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_QFACTOR, UVC_GET_MAX, outputValues, desc);
				cout << "Maximum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_QFACTOR, UVC_GET_CUR, outputValues, desc);
				cout << "Current Factor value : " << (unsigned int)outputValues << endl;

				cout << "Enter MJPG Qfactor value : ";
				cin >> set_value;
				if (!set_xu_cmd(ECAM22_XU_QFACTOR, set_value, desc))
					perror("Set failed");

#endif
				break;
			case ECAM22_XU_BITRATE:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << endl << "Camera " << UVCExtension[choice].second << " Values:: " << endl;

				cout << '\t' << "Minimum Value: " << (int)uCtrlMinValue << endl;
				cout << '\t' << "Maximum Value: " << (int)uCtrlMAxValue << endl;
				cout << '\t' << "SteppingDelta: " << (int)uCtrlStepValue << endl;    //incrementing scale Value between min and max value
				cout << '\t' << "Current Value: " << (int)uCtrlCurrentValue << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is set with the Value : " << value << endl;

#elif __linux__
				get_xu_cmd(ECAM22_XU_BITRATE, UVC_GET_MIN, outputValues, desc);
				cout << "Minimum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_BITRATE, UVC_GET_MAX, outputValues, desc);
				cout << "Maximum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_BITRATE, UVC_GET_CUR, outputValues, desc);
				cout << "Current BitRate value : " << (unsigned int)outputValues << endl;

				cout << "Enter H264 BitRate value : ";
				cin >> set_value;
				if (!set_xu_cmd(ECAM22_XU_BITRATE, set_value, desc))
					perror("Set failed");

#endif
				break;
			case ECAM22_XU_H264QUALITY:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << endl << "Camera " << UVCExtension[choice].second << " Values:: " << endl;

				cout << '\t' << "Minimum Value: " << (int)uCtrlMinValue << endl;
				cout << '\t' << "Maximum Value: " << (int)uCtrlMAxValue << endl;
				cout << '\t' << "SteppingDelta: " << (int)uCtrlStepValue << endl;    //incrementing scale Value between min and max value
				cout << '\t' << "Current Value: " << (int)uCtrlCurrentValue << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is set with the Value : " << value << endl;
#elif __linux__
				get_xu_cmd(ECAM22_XU_H264QUALITY, UVC_GET_MIN, outputValues, desc);
				cout << "Minimum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_H264QUALITY, UVC_GET_MAX, outputValues, desc);
				cout << "Maximum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_H264QUALITY, UVC_GET_CUR, outputValues, desc);
				cout << "Current Quality value : " << (unsigned int)outputValues << endl;

				cout << "Enter H264 Quality value : ";
				cin >> set_value;
				if (!set_xu_cmd(ECAM22_XU_H264QUALITY, set_value, desc))
					perror("Set failed");
#endif
				break;
			case ECAM22_XU_CBR:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << "\n\tCurrent Mode : " << (uCtrlCurrentValue ? "ON" : "OFF") << endl;

				cout << "CBR Mode : " << endl << "\t0 - OFF" << endl << "\t1 - ON" << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is " << (value ? "ON" : "OFF") << endl;

#elif __linux__
				if (!get_xu_cmd(ECAM22_XU_CBR, UVC_GET_MIN, outputValues, desc))
				{
					perror("get cur CBR properties failes");
					return false;
				}
				min = outputValues;
				if (!get_xu_cmd(ECAM22_XU_CBR, UVC_GET_MAX, outputValues, desc))
				{
					perror("get cur CBR properties failes");
					return false;
				}
				max = outputValues;
				if (!get_xu_cmd(ECAM22_XU_CBR, UVC_GET_CUR, outputValues, desc))
				{
					perror("get cur CBR properties failes");
					return false;
				}
				cout << "\n\tCurrent Mode : " << (outputValues ? "ON" : "OFF") << endl;

				cout << "CBR Mode : " << endl << "\t0 - OFF" << endl << "\t1 - ON" << endl;
				while (true)
				{
					cout << endl << "Enter a Valid value to Set CBR mode : " << '\t';
					cin >> set_value;
					if ((set_value >= min) && (set_value <= max))
						break;
					cout << "Certain Conditions Not met; Please " << endl;
				}

				if (!set_xu_cmd(ECAM22_XU_CBR, set_value, desc))
				{
					perror("Set CBR mode failed");
					return false;
				}

				cout << " CBR Mode is " << (set_value ? "ON" : "OFF") << endl;
#endif
				break;
			case ECAM22_XU_EXPOSURE_ROI_MOUSE_CLICK:
				cout << "\n\tNot Yet implemented in FW" << endl;
				break;
			case ECAM22_XU_NOISE_REDUCION:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << endl << "Camera " << UVCExtension[choice].second << " Values:: " << endl;

				cout << '\t' << "Minimum Value: " << (int)uCtrlMinValue << endl;
				cout << '\t' << "Maximum Value: " << (int)uCtrlMAxValue << endl;
				cout << '\t' << "SteppingDelta: " << (int)uCtrlStepValue << endl;    //incrementing scale Value between min and max value
				cout << '\t' << "Current Value: " << (int)uCtrlCurrentValue << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is set with the Value : " << value << endl;


#elif __linux__
				get_xu_cmd(ECAM22_XU_NOISE_REDUCION, UVC_GET_CUR, outputValues, desc);
				cout << "Current Denoise value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_NOISE_REDUCION, UVC_GET_MIN, outputValues, desc);
				cout << "Minimum Value : " << (unsigned int)outputValues << endl;
				get_xu_cmd(ECAM22_XU_NOISE_REDUCION, UVC_GET_MAX, outputValues, desc);
				cout << "maximum Value : " << (unsigned int)outputValues << endl;
				cout << "Enter new value ";
				cin >> set_value;
				if (!set_xu_cmd(ECAM22_XU_NOISE_REDUCION, set_value, desc))
					perror("Set Failed");
#endif
				break;
			case ECAM22_XU_DEWARPING:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << "\n\tCurrent Mode : " << (uCtrlCurrentValue ? "ON" : "OFF") << endl;

				cout << "\nDewarp Modes : " << endl << "\t0 - OFF" << endl << "\t1 - ON" << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is " << (value ? "ON" : "OFF") << endl;

#elif __linux__
				cout << endl << "Camera " << UVCExtension[choice].second << " Values:: " << endl;

				get_xu_cmd(ECAM22_XU_DEWARPING, UVC_GET_CUR, outputValues, desc);
				if (outputValues == 1)
					cout << " Dewarp Mode ON" << endl;
				else
					cout << "Dewarp Mode OFF" << endl;

				get_xu_cmd(ECAM22_XU_DEWARPING, UVC_GET_MIN, outputValues, desc);
				cout << "Minimum Value : " << (unsigned int)outputValues << "- OFF" << endl;
				get_xu_cmd(ECAM22_XU_DEWARPING, UVC_GET_MAX, outputValues, desc);
				cout << "Maximum Value : " << (unsigned int)outputValues << "- ON" << endl;

				cout << "Enter mode to be set : ";
				cin >> set_value;
				if (!set_xu_cmd(ECAM22_XU_DEWARPING, set_value, desc))
					perror("Set failed");
#endif
				break;
			case ECAM22_XU_EXPOSURE_ROI_MODE:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << "\n\tCurrent Mode : ";

				if (uCtrlCurrentValue == 1) cout << "MANUAL" << endl;
				else if (uCtrlCurrentValue == 2)	cout << "DISABLE" << endl;
				else cout << "FULL" << endl;

				cout << "ROI modes " << endl << "\t0 - FULL" << endl << "\t1 - MANUAL" << endl << "\t2 - DISABLE" << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is ";

				if (value == 1) cout << "MANUAL" << endl;
				else if (value == 2)	cout << "DISABLE" << endl;
				else cout << "FULL" << endl;
#elif __linux__

				cout << "ROI modes " << endl << '\t' << "0 - FULL" << endl << '\t' << "1 - MANUAL" << endl << '\t' << "2 - DISABLE" << endl;

				get_xu_cmd(ECAM22_XU_EXPOSURE_ROI_MODE, UVC_GET_CUR, outputValues, desc);
				cout << "Current ROI mode : ";
				if (outputValues == 0)
					cout << " FULL " << endl;
				else if (outputValues == 1)
					cout << " MANUAL " << endl;
				else
					cout << " DISABLE " << endl;

				cout << "Enter the roi mode to be set : ";
				cin >> set_value;
				if (!set_xu_cmd(ECAM22_XU_EXPOSURE_ROI_MODE, set_value, desc))
					perror("Set failed");
				if (set_value == 1)
				{
					get_xu_cmd(ECAM22_XU_EXPOSURE_ROI_WINSIZE, UVC_GET_MIN, outputValues, desc);
					cout << "Minimum Value : " << (unsigned int)outputValues << endl;
					get_xu_cmd(ECAM22_XU_EXPOSURE_ROI_WINSIZE, UVC_GET_MAX, outputValues, desc);
					cout << "Maximum Value : " << (unsigned int)outputValues << endl;
					get_xu_cmd(ECAM22_XU_EXPOSURE_ROI_WINSIZE, UVC_GET_CUR, outputValues, desc);
					cout << "Current ROI window size :" << (unsigned int)outputValues << endl;

					cout << "Enter the window size : ";
					cin >> set_value;

					if (!set_xu_cmd(ECAM22_XU_EXPOSURE_ROI_WINSIZE, set_value, desc))
						perror("Set failed");
				}
#endif
				break;
			case ECAM22_XU_HDR:
#ifdef _WIN32
				if (!(GetVideoControlRangeECAM22(UVCExtension[choice].first, &uCtrlMinValue, &uCtrlMAxValue, &uCtrlStepValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				if (!(GetVideoControlRequestECAM22(UVCExtension[choice].first, &uCtrlCurrentValue)))
				{
					cout << UVCExtension[choice].second << " Properties Couldn't be Retrieved for the Camera Connected" << endl;
					return false;
				}

				cout << "\n\tCurrent Mode : ";

				if (uCtrlCurrentValue == 0)
					cout << "HDR OFF" << endl;
				else if (uCtrlCurrentValue == 1)
					cout << "HDR 2x" << endl;

				cout << "\nHDR modes \n" << "\t0 - HDR OFF" << "\n\t1 - HDR 2x" << endl;

				while (true)
				{
					cout << endl << "Enter a Valid value to Set " << UVCExtension[choice].second << " : " << '\t';
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
					if ((value >= uCtrlMinValue) && (value <= uCtrlMAxValue) && (((value) % (uCtrlStepValue)) == 0))
					{
						break;
					}
					cout << endl << "Certain Conditions Not met; Please ";
				}

				if (!(SetVideoControlECAM22(UVCExtension[choice].first, value)))
				{
					cout << endl << "Camera " << UVCExtension[choice].second << " is Not Set" << endl;
					return false;
				}

				cout << UVCExtension[choice].second << " Mode is ";

				if (value == 0)
					cout << "HDR OFF" << endl;
				else if (value == 1)
					cout << "HDR 2x" << endl;

#elif __linux__
				if (!get_xu_cmd(ECAM22_XU_HDR, UVC_GET_MIN, outputValues, desc))
				{
					cout << "get MIN HDR properties failes" << endl;
					return false;
				}
				cout << "Minimum Value : " << (unsigned int)outputValues << endl;

				if (!get_xu_cmd(ECAM22_XU_HDR, UVC_GET_MAX, outputValues, desc))
				{
					cout << "get Max HDR properties failes" << endl;
					return false;
				}
				cout << "maximum Value : " << (unsigned int)outputValues << endl;

				if (!get_xu_cmd(ECAM22_XU_HDR, UVC_GET_CUR, outputValues, desc))
				{
					cout << "get curr HDR properties failes" << endl;
					return false;
				}

				cout << "Current HDR: " << (outputValues ? "HDR x2" : "HDR OFF") << endl;

				cout << "\nHDR modes \n" << "\t0- HDR OFF" << "\n\t1- HDR 2x" << endl;

				cout << "Enter mode to be set : ";
				cin >> set_value;

				if (!set_xu_cmd(ECAM22_XU_HDR, set_value, desc))
				{
					perror("Set HDR mode failed");
					return false;
				}

				cout << " HDR mode set to " << (set_value ? "HDR x2" : "HDR OFF") << endl;
#endif
				break;
			}
		}
		while (true)
		{
			cout << endl << "Enter y/Y to Continue or n/N to dis-Continue: " << '\t';
			scanf(" %c", &dilemma);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
			if (dilemma == 'y' || dilemma == 'Y')
				goto REPEAT;
			if (dilemma == 'n' || dilemma == 'N')
				break;
		}
	}
	return true;
}

//Explore Camera Properties

bool exploreCam()
{
	while (true)
	{
		int choice = -1;
		cout << endl << '\t' << "0 - Exit" << endl;
		cout << '\t' << "1 - Back" << endl;
#ifdef _WIN32
		if (bRecStarted)
		{
			cout << '\t' << "2 - Stop Recording" << endl;

			while ((choice < 0) || (choice > 2))
			{
				printf("\n Pick a Relevant Choice to Configure Particular Camera Properties : \t");
				scanf("%d", &choice);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
			}
		}
		else
		{
#endif
			cout << '\t' << "2 - Configure Camera Format/Resolution" << endl;
			cout << '\t' << "3 - Configure UVC Settings" << endl;
			cout << '\t' << "4 - Capture Still Images" << endl;
			cout << '\t' << "5 - Record Video" << endl;

			if (bOpenHID)
			{
				cout << '\t' << "6 - Extension Settings" << endl;
				while ((choice < 0) || (choice > 6))
				{
					printf("\n Pick a Relevant Choice to Configure Particular Camera Properties : \t");
					scanf("%d", &choice);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				}
			}
			else
			{
				while ((choice < 0) || (choice >= 6))
				{
					printf("\n Pick a Relevant Choice to Configure Particular Camera Properties : \t");
					scanf("%d", &choice);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				}
			}
#ifdef _WIN32
		}
#endif
		//    int node;
		switch (choice)
		{
		case EXIT:
#ifdef _WIN32

			if (deinitextensionunit())
			{
				t.detach();
			}
			MFDeInit();
			bSwitch = true;
			if (cap.isOpened())
				cap.release();

#elif __linux__

			bPreviewSet(1, false);
			if (closeHID())
				destroyAllWindows();
			if (cap.isOpened())
				cap.release();
			if (cap_h264.isOpened())
				cap_h264.release();
#endif

			exit(0);

		case 1:
#ifdef _WIN32
			if (bRecStarted)
			{
				bRecStarted = false;
				MFStopRecording();
			}

			bSwitch = true;
#endif
			if (!listDevices())
			{
				cout << endl << "List Devices Information failed" << endl;
				return false;
			}
			cout << endl << "Connected Devices were Listed" << endl;
			break;

		case 2:
#ifdef _WIN32
			if (bRecStarted)
			{
				bRecStarted = false;
				MFStopRecording();
				cout << "Recording Stoped..." << endl;
			}
			else
			{
#endif
				choose_node();
				if (!configFormats(node_choice))
				{
					cout << endl << "Format Configuration is Failed" << endl;
					perror("getFormat failed");
					return false;
				}
				cout << endl << "Format Configuration is done" << endl;
#ifdef _WIN32
			}
#endif
			break;

		case 3:
			choose_node();
			if (!configUVCSettings(node_choice))
			{
				cout << endl << "UVC Settings Configuration is Failed" << endl;
				return false;
			}
			cout << endl << "UVC Settings Configuration is Done" << endl;
			break;

		case 4:
			if (!captureStill())
			{
				cout << endl << "Still Capture Failed" << endl;
			}
			break;
		case 5:
#ifdef _WIN32
			if (!bRecStarted)
				Recordvideo();
#elif __linux__
			record_menu();
#endif
			break;
		case 6:
			choose_node();
			if (!hidProp(fd_choice))
			{
				cout << endl << "HID Properties Configuration is Failed" << endl;
				return false;
			}
			cout << endl << "HID Properties Configuration is Done" << endl;
			break;
		}
	}

	return true;
}

#ifdef __linux__
bool set_xu_cmd(__u8 controlId, uint16_t setVal, int *desc)
{
	struct uvc_xu_control_query xquery;
	__u16 size = 0;
	int ret = -1;
	memset(&xquery, 0, sizeof(xquery));
	xquery.query = UVC_GET_LEN;
	xquery.size = 2;
	xquery.selector = controlId;
	xquery.unit = EXTENSION_UNIT_ID;
	xquery.data = (__u8 *)&size;
	ret = ioctl(*desc, UVCIOC_CTRL_QUERY, &xquery);
	if (ret<0)
	{
		perror("ioctl Failed");
		return false;
	}

	// To set the current value to the camera
	xquery.query = UVC_SET_CUR;
	xquery.size = size;
	xquery.selector = controlId;
	xquery.unit = EXTENSION_UNIT_ID;
	xquery.data = (uint8_t *)&setVal;

	ret = ioctl(*desc, UVCIOC_CTRL_QUERY, &xquery);
	if (ret<0)
	{
		perror("ioctl Failed");
		return false;
	}
	return true;
}
bool get_xu_cmd(__u8 controlId, __u8 queryType, __u8 &outputValues, int *desc)
{
	struct uvc_xu_control_query xquery;
	__u16 size = 0;
	int ret = -1;
	memset(&xquery, 0, sizeof(xquery));
	xquery.query = UVC_GET_LEN;
	xquery.size = 2;
	xquery.selector = controlId;
	xquery.unit = EXTENSION_UNIT_ID;
	xquery.data = (__u8 *)&size;
	ret = ioctl(*desc, UVCIOC_CTRL_QUERY, &xquery);
	if (ret<0)
	{
		perror("ioctl Failed");
		return false;
	}

	memset(&xquery, 0, sizeof(xquery));
	// get value from camera
	xquery.query = queryType;
	xquery.size = size;
	xquery.selector = controlId;
	xquery.unit = EXTENSION_UNIT_ID;
	xquery.data = (unsigned char *)(&outputValues);

	ret = ioctl(*desc, UVCIOC_CTRL_QUERY, &xquery);
	if (ret<0)
	{
		perror("ioctl Failed");
		return false;
	}
	__u8 *val = xquery.data;

	outputValues = *val;
	return true;
}

#endif

#ifdef __linux__
bool record_menu()
{
	__u8 outputValues;
	char c;
	int choice;
	if (!cap_h264.isOpened())
	{
		if (!cap_h264.open(cam_list[camId - 1].first.o_index))
			cout << "Failed to Open " << cam_list[camId - 1].first.o_device_name << " Device" << endl;

		std::string cam_name = String(cam_list[camId - 1].first.o_device_path);
		const char * path = cam_name.c_str();

		cam_list[camId - 1].first.fd = ::open(path, O_RDWR | O_NONBLOCK, 0);
		if (cam_list[camId - 1].first.fd < 0)
			perror("Coudnt open h264_fd");
	}

	// Zoom, Tilt, Pan Settings.
	cap_h264.get(CV_CAP_PROP_ZOOM, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue);
	cam_list[camId - 1].first.Zoom = currentValue;
	cap_h264.get(CV_CAP_PROP_TILT, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue);
	cam_list[camId - 1].first.Tilt = currentValue;
	cap_h264.get(CV_CAP_PROP_PAN, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue);
	cam_list[camId - 1].first.Pan = currentValue;

	cap.get(CV_CAP_PROP_ZOOM, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue);
	cam_list[camId - 1].second.Zoom = currentValue;
	cap.get(CV_CAP_PROP_TILT, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue);
	cam_list[camId - 1].second.Tilt = currentValue;
	cap.get(CV_CAP_PROP_PAN, minimum, maximum, steppingDelta, supportedMode, currentValue, currentMode, defaultValue);
	cam_list[camId - 1].second.Pan = currentValue;

	if ((cam_list[camId - 1].second.Zoom != cam_list[camId - 1].first.Zoom) || (cam_list[camId - 1].second.Tilt != cam_list[camId - 1].first.Tilt) || (cam_list[camId - 1].second.Pan != cam_list[camId - 1].first.Pan))
	{
		cout << "\n\tNOTE: Preview & Record node Settings are different." << endl;

		while (true)
		{
			cout << endl << "Enter y/Y to Apply or n/N to do not Apply: " << '\t';
			scanf(" %c", &dilemma);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
			if (dilemma == 'y' || dilemma == 'Y')
			{
				if (cap_h264.isOpened())
				{
					if (!cap_h264.set(CV_CAP_PROP_ZOOM, cam_list[camId - 1].second.Zoom, MANUAL))
						cout << "Failed to set Zoom Value" << endl;
					if (!cap_h264.set(CV_CAP_PROP_TILT, cam_list[camId - 1].second.Tilt, MANUAL))
						cout << "Failed to set Tilt Value" << endl;
					if (!cap_h264.set(CV_CAP_PROP_PAN, cam_list[camId - 1].second.Pan, MANUAL))
						cout << "Failed to set Pan Value" << endl;
				}
				else
					cout << "Device is not Opned" << endl;
				break;
			}
			else if (dilemma == 'n' || dilemma == 'N')
				break;
		}
	}
	cout << " Recording Started " << endl;
	stop_record = false;
	pthread_create(&threadH264, NULL, record_h264, NULL);

	cout << "\n\t0-Exit" << endl;
	cout << "\t1-Back" << endl;
	cout << "\t2-Stop Recording" << endl;
	cout << "\nEnter your choice : \t";
	cin >> choice;
	switch (choice)
	{
	case 0:pthread_kill(threadH264, SIGINT);
		bPreviewSet(1, false);
		if (closeHID())
			destroyAllWindows();
		if (cap_h264.isOpened())
			cap_h264.release();
		exit(0);
	case 1:
	case 2:stop_record = true;
		pthread_join(threadH264, NULL);
	}
	return true;
}
#endif

void choose_node()
{
	cout << "\nSelect Stream :" << endl;
	cout << "\n\t1. " << cam_list[camId - 1].first.o_device_name << "\n\t2. " << cam_list[camId - 1].second.o_device_name << endl;
	cout << "\nEnter Stream : \t";
	cin >> node;

#ifdef __linux__
	if (node == 1)
	{
		if (cap_h264.isOpened())
			cap_h264.release();
		if (!cap_h264.open(cam_list[camId - 1].first.o_index))
			cout << "Failed to Open " << cam_list[camId - 1].first.o_device_name << " Device" << endl;
		node_choice = &cap_h264;

		std::string str = String(cam_list[camId - 1].first.o_device_path);
		const char * path = str.c_str();

		cam_list[camId - 1].first.fd = ::open(path, O_RDWR | O_NONBLOCK, 0);
		if (cam_list[camId - 1].first.fd < 0)
			perror("Coudnt open h264_fd");

		fd_choice = &cam_list[camId - 1].first.fd;
	}
	else
	{
		node_choice = &cap;
		fd_choice = &cam_list[camId - 1].second.fd;
		if (node_choice == NULL || fd_choice == NULL)
			cout << "ptr NULL" << endl;
	}
#endif

#ifdef _WIN32
	if (node == 2)
	{
		if (!cap_h264.open(cam_list[camId - 1].second.o_index))
			cout << "Failed to Open " << cam_list[camId - 1].second.o_device_name << " Device" << endl;
		node_choice = &cap_h264;

		deinitextensionunit();

		tDevicePath = new TCHAR[cam_list[camId - 1].second.o_device_path.size() + 1];
		copy(cam_list[camId - 1].second.o_device_path.begin(), cam_list[camId - 1].second.o_device_path.end(), tDevicePath);
		bOpenHID = initextensionunit(tDevicePath);
		if (!bOpenHID)
			cout << "initextensionunit failed" << endl;
	}
	else
	{
		node_choice = &cap;

		deinitextensionunit();

		tDevicePath = new TCHAR[cam_list[camId - 1].first.o_device_path.size() + 1];
		copy(cam_list[camId - 1].first.o_device_path.begin(), cam_list[camId - 1].first.o_device_path.end(), tDevicePath);
		bOpenHID = initextensionunit(tDevicePath);
		if (!bOpenHID)
			cout << "initextensionunit failed" << endl;
	}
#endif
}
#ifdef __linux__

void *record_h264(void *arg)
{
	if (cap_h264.isOpened())
	{
		char c;
		char file_name[260];

		int fps = 30;
		int frame_width = cap_h264.get(CV_CAP_PROP_FRAME_WIDTH);
		int frame_height = cap_h264.get(CV_CAP_PROP_FRAME_HEIGHT);
		Size S = Size(frame_height, frame_width); // Declare Size structure
		int fourcc = VideoWriter::fourcc('H', '2', '6', '4');
		VideoWriter outputVideo;
		sprintf(file_name, "Hyperyon_Cam_%dx%d", frame_width, frame_height);
		timestamp_string(file_name);
		outputVideo.open(file_name, fourcc, fps, S, true);
		while (1)
		{
			if (cap_h264.isOpened())
			{
				cap_h264 >> Frame_h264;
				if (!Frame_h264.empty())
					outputVideo.write(Frame_h264);
			}
			if (stop_record)
				break;
		}
		outputVideo.release();
		cout << "Saved recording in current directory" << endl;
	}
}

void timestamp_string(char * myString)
{
	char     timestamp[16];
	time_t    caltime;
	struct tm * broketime;
	// find current time, convert to broken-down time
	time(&caltime);
	broketime = localtime(&caltime);
	// append timestamp in the format "_yymmdd_hhmmss"
	strftime(timestamp, 16, "_%y%m%d_%H%M%S", broketime);
	strcat(myString, timestamp);
	strcat(myString, ".avi");
	return;
}

#endif
