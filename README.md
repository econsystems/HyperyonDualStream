# HyperyonDualStream 

## Videoio Module

* This Videoio Module can be supported for both Opencv version 3.3.1 and 3.4.1 with both Linux and Windows OS.

## HyperyonCam Command Line Application

* HyperyonCam command line application is an opencv based program for exploring the possibilities of e-con Systems's Hyperyon Dual Stream camera.It can be used to access the UVC settings,Extension settings, Streaming, Image capture and video recording of the camera in both Windows and Linux.
	Here, one device is enumerated as two nodes.One strictly for previewing(Non H264) and other(H264) for recording purpose.
Both nodes can be configured as required.(Note: some UVC settings are common for both nodes so altering one will reflect on other node also.Refer manual for more info). 

## Features

* Two streaming formats UYVY, MJPG.
* Seperate settings can be used for previewing and recording.
* Video recording is done for Record node (H264 format).
* Capture still in raw as well as jpeg format.

## How to Use

* OpenCV can be downloaded from [here](https://github.com/opencv/opencv)
```
	$ cd opencv

	$ git checkout <opencv version>
```

* Replace Videoio module from OpenCV with [this videoio module](https://github.com/econsystems/HyperyonDualStream/tree/master/Source)

* Build OpenCV using [this Installation manual](https://github.com/econsystems/HyperyonDualStream/tree/master/Documents)

* HyperyonCam command line application can be downloaded [from here](https://github.com/econsystems/HyperyonDualStream/tree/master/Source), which is used to access OpenCV APIs

* Run Sample application using [this user manual](https://github.com/econsystems/HyperyonDualStream/tree/master/Documents)

## Supported Camera's

	* e-CAM22_USB (Dual Stream)

## Release note
* Windows video recording using FFMPEG (libavcodec).

## Release
* HyperyonCam v1.0.1		-	08-Sep-20
* HyperyonCam v1.0.0		-	12-Aug-20

