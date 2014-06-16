/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Antons Rebguns
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef VIDERESTEREO1394_H
#define VIDERESTEREO1394_H

#include <stdexcept>
#include <cstring>

#include <dc1394/dc1394.h>
#include <sensor_msgs/image_encodings.h>
#include <videre_stereo_cam/stereoimage.h>

// Videre offsets
#define VIDERE_LOCAL_BASE                     0xF0000UL
#define VIDERE_PARAM1_OFFSET                  0x0400
#define VIDERE_CALIB_OFFSET                   0x0800
#define VIDERE_CALIB_SIZE                     0x1000           //  4 KB
#define VIDERE_USER_OFFSET                    0x1800
#define VIDERE_USER_SIZE                      0x1000           //  4 KB
#define VIDERE_DOWNLOAD_OFFSET                0x4000
#define VIDERE_DOWNLOAD_SIZE                  0x3C00           // 15 KB
#define VIDERE_BLOCK_SIZE                     0x200            // 512 B block size
#define VIDERE_UPGRADE_CONFIG                 0x7A00           // rom config values to be saved
#define VIDERE_UPGRADE_PARAMS                 0x7B00           // local param storage
#define VIDERE_UPGRADE_OFFSET                 0x7BF8           // where magic num is written
#define VIDERE_SCRATCHPAD_OFFSET              0x7C00
#define VIDERE_SCRATCHPAD_SIZE                0x80             // 128 B
#define VIDERE_LOCAL_PARAM_BASE               0xFF000UL
#define VIDERE_CAM_STORE_MAGIC_NUM            0x5A01F687UL
#define VIDERE_CAM_FW_LEVEL_OFFSET            (1*4)
#define VIDERE_CAM_LEVEL_OFFSET               (2*4)
#define VIDERE_CAM_ID_OFFSET                  (3*4)
#define VIDERE_CAM_BLACK_OFFSET               (4*4)
#define VIDERE_CAM_START_ROW_OFFSET           (5*4)
#define VIDERE_CAM_START_COL_OFFSET           (6*4)
#define VIDERE_CAM_IMAGER_SIZE_OFFSET         (7*4)
#define VIDERE_CAM_FRAME_DIV_OFFSET           (8*4)
#define VIDERE_CAM_50HZ_OFFSET                (9*4)
#define VIDERE_CAM_PROC_OFFSET                (10*4)
#define VIDERE_CAM_PROC_THRESH_OFFSET         (11*4)

// define some Videre modes
#define VIDERE_STEREO_1280x960 DC1394_VIDEO_MODE_1280x960_YUV422
#define VIDERE_STEREO_1024x768 DC1394_VIDEO_MODE_1024x768_YUV422
#define VIDERE_STEREO_640x480 DC1394_VIDEO_MODE_640x480_YUV422
#define VIDERE_STEREO_512x384 DC1394_VIDEO_MODE_512x384_YUV422
#define VIDERE_STEREO_320x240 DC1394_VIDEO_MODE_320x240_YUV422

// STOC modes
typedef enum
{
    PROC_MODE_OFF = 0,
    PROC_MODE_NONE,
    PROC_MODE_TEST,
    PROC_MODE_RECTIFIED,
    PROC_MODE_DISPARITY,
    PROC_MODE_DISPARITY_RAW
} videre_proc_mode_t;


class VidereStereoDriverException : public std::runtime_error
{
public:
    VidereStereoDriverException(const char* msg) : std::runtime_error(msg) {}
};


class VidereStereoDriver
{
public:
    VidereStereoDriver(uint64_t guid, size_t bufferSize = 8);
    ~VidereStereoDriver();

    char* getVendor();
    char* getModel();
    dc1394video_modes_t* getModes();

    void setFormat(dc1394video_mode_t video = DC1394_VIDEO_MODE_640x480_MONO8,
                   dc1394framerate_t fps = DC1394_FRAMERATE_30,
                   dc1394speed_t speed = DC1394_ISO_SPEED_400);

    void initialize_camera(uint64_t guid);
    void fini();
    void start();
    void stop();
    const char* getModeString(dc1394video_mode_t mode);

    uint64_t guid;
    uint32_t imFirmware, camFirmware, stocFirmware;
    bool isSTOC, isVidereStereo, isVidere, isColor; // true if a STOC, Videre stereo, color device

    // gets the next image, with timeout
    bool getImage(int ms);
    void setCapturePolicy(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    // general DC1394 interface
    bool hasFeature(dc1394feature_t feature);
    void getFeatureBoundaries(dc1394feature_t feature, uint32_t& min, uint32_t& max);
    void setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2 = 0);
    void setFeatureAbsolute(dc1394feature_t feature, float value);
    void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode);

    // particular features of importance
    void setExposure(int val, bool isauto);
    void setGain(int val, bool isauto);
    void setBrightness(int val, bool isauto);
    void setWhiteBalance(int blue_val, int red_val, bool isauto);

    // feature boundaries
    uint32_t expMax, expMin;
    uint32_t gainMax, gainMin;
    uint32_t brightMax, brightMin;
    uint32_t whiteBalanceMin, whiteBalanceMax;
    bool setMaxAutoVals(int exp, int gain); // set max for auto gain and exposure algorithm

    // low-level register access
    // implicitly assumes CCR base, so that DCAM register are at offsets
    //   e.g., for EXPOSURE use 0x804
    void setRegister(uint64_t offset, uint32_t value);
    uint32_t getRegister(uint64_t offset);

    bool setProcMode(videre_proc_mode_t mode);

    bool setCompanding(bool on);    // bring up low light levels
    bool setHDR(bool on);           // high dynamic range

    char *getParameters();          // download from device
    char *retParameters();          // just return current param string
    bool putParameters(char *p);    // just set current param string, handle buffering
    bool setParameters();           // upload to device
    bool setSTOCParams(uint8_t *cbuf, int cn, // upload to STOC device
                       uint8_t *lbuf, int ln, // STOC firmware, left and right warp tables
                       uint8_t *rbuf, int rn);

    int getIncRectTable(uint8_t *buf);	// make a warp table for a STOC device

    // Videre camera de-interlacing
    void stereoDeinterlace(uint8_t *src, uint8_t **d1, size_t *s1, uint8_t **d2, size_t *s2);
    void stereoDeinterlace2(uint8_t *src, uint8_t **d1, size_t *s1, int16_t **d2, size_t *s2);

    // stereo image and processing
    StereoData *stIm;

    // gets the next image, with timeout
    void fillImageMsg(sensor_msgs::Image& img, std::string enc, uint32_t bpp);
    void fillDisparityMsg(stereo_msgs::DisparityImage& img);

    // processing parameters
    bool setTextureThresh(int thresh);
    bool setUniqueThresh(int thresh);
    bool setHoropter(int thresh);
    bool setSpeckleSize(int size);
    bool setSpeckleDiff(int diff);

private:
    bool started;
    size_t bufferSize;                  // number of DMA buffers

    dc1394_t* dcRef;
    dc1394camera_t *dcCam;              // the camera object

    dc1394video_modes_t camModes;       // valid modes
    dc1394capture_policy_t camPolicy;   // current capture policy
    dc1394video_frame_t* camFrame;      // current captured frame
    videre_proc_mode_t procMode;        // STOC mode, if applicable
    dc1394video_mode_t videoMode;
    color_coding_t rawType;             // what type of raw image we receive

    void cleanup();
    void setRawType();
    bool store_eeprom_bytes(int addr, uint8_t *buf, int count);
};

#endif
