/*M///////////////////////////////////////////////////////////////////////////////////////
//
// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.
//
// Copyright (C) 2014, Itseez, Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
//M*/

#ifndef _CAP_DSHOW_HPP_
#define _CAP_DSHOW_HPP_

#include "precomp.hpp"

#ifdef HAVE_DSHOW

class videoInput;
namespace cv
{

class VideoCapture_DShow : public IVideoCapture
{
public:
    VideoCapture_DShow();
    VideoCapture_DShow(int index);
    virtual ~VideoCapture_DShow();

    virtual double getProperty(int propIdx) const;
    virtual bool setProperty(int propIdx, long propVal);

    virtual bool getDevices(int &devices);
    virtual bool getDeviceInfo(int index, String &deviceName, String &vid, String &pid, String &devicePath);
    virtual bool getFormats(int &formats);
    virtual bool getFormatType(int formats, String &formatType, int &width, int &height, int &fps);
    virtual bool getVideoProperty(int propIdx, int &min, int &max, int &steppingDelta, int &supportedMode, int &currentValue, int &currentMode, int &defaultValue);
    virtual bool setVideoProperty(int propIdx, int value, int mode);
    virtual bool grabFrame();
    virtual bool retrieveFrame(int outputType, OutputArray frame);
    virtual int getCaptureDomain();
    virtual bool isOpened() const;
protected:
    void open(int index);
    void close();

	int m_index, m_width, m_height;
	int m_fourcc_index;
    int m_widthSet, m_heightSet, m_FourccSet;
	bool m_ConvertRGB;
    static videoInput g_VI;
};

}

#endif //HAVE_DSHOW
#endif //_CAP_DSHOW_HPP_
