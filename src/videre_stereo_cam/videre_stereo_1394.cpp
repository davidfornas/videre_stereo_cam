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

#include <cstring>
#include <cstdio>
#include <errno.h>

#include <ros/console.h>
#include <ros/assert.h>

#include <dc1394/dc1394.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <videre_stereo_cam/stereoimage.h>
#include <videre_stereo_cam/videre_stereo_1394.h>
#include <videre_stereo_cam/videre_dc1394_control.h>

#define PRINTF(a...) ROS_INFO(a)

#define CHECK_READY()                                                               \
    if (!dcRef)                                                                     \
    {                                                                               \
        char msg[256];                                                              \
        snprintf(msg, 256, "Tried to call %s before calling init()", __FUNCTION__); \
        throw VidereStereoDriverException(msg);                                     \
    }

#define CHECK_ERR(fnc, amsg)                                                  \
    {                                                                         \
        dc1394error_t err = fnc;                                              \
        if (err != DC1394_SUCCESS)                                            \
        {                                                                     \
            char msg[256];                                                    \
            snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg); \
            throw VidereStereoDriverException(msg);                           \
        }                                                                     \
    }

#define CHECK_ERR_CLEAN(fnc, amsg)                                            \
    {                                                                         \
        dc1394error_t err = fnc;                                              \
        if (err != DC1394_SUCCESS)                                            \
        {                                                                     \
            cleanup();                                                        \
            char msg[256];                                                    \
            snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg); \
            throw VidereStereoDriverException(msg);                           \
        }                                                                     \
    }

static const char *modestrings[DC1394_VIDEO_MODE_NUM] =
{
    "DC1394_VIDEO_MODE_160x120_YUV444",
    "DC1394_VIDEO_MODE_320x240_YUV422",
    "DC1394_VIDEO_MODE_640x480_YUV411",
    "DC1394_VIDEO_MODE_640x480_YUV422",
    "DC1394_VIDEO_MODE_640x480_RGB8",
    "DC1394_VIDEO_MODE_640x480_MONO8",
    "DC1394_VIDEO_MODE_640x480_MONO16",
    "DC1394_VIDEO_MODE_800x600_YUV422",
    "DC1394_VIDEO_MODE_800x600_RGB8",
    "DC1394_VIDEO_MODE_800x600_MONO8",
    "DC1394_VIDEO_MODE_1024x768_YUV422",
    "DC1394_VIDEO_MODE_1024x768_RGB8",
    "DC1394_VIDEO_MODE_1024x768_MONO8",
    "DC1394_VIDEO_MODE_800x600_MONO16",
    "DC1394_VIDEO_MODE_1024x768_MONO16",
    "DC1394_VIDEO_MODE_1280x960_YUV422",
    "DC1394_VIDEO_MODE_1280x960_RGB8",
    "DC1394_VIDEO_MODE_1280x960_MONO8",
    "DC1394_VIDEO_MODE_1600x1200_YUV422",
    "DC1394_VIDEO_MODE_1600x1200_RGB8",
    "DC1394_VIDEO_MODE_1600x1200_MONO8",
    "DC1394_VIDEO_MODE_1280x960_MONO16",
    "DC1394_VIDEO_MODE_1600x1200_MONO16",
    "DC1394_VIDEO_MODE_EXIF",
    "DC1394_VIDEO_MODE_FORMAT7_0",
    "DC1394_VIDEO_MODE_FORMAT7_1",
    "DC1394_VIDEO_MODE_FORMAT7_2",
    "DC1394_VIDEO_MODE_FORMAT7_3",
    "DC1394_VIDEO_MODE_FORMAT7_4",
    "DC1394_VIDEO_MODE_FORMAT7_5",
    "DC1394_VIDEO_MODE_FORMAT7_6",
    "DC1394_VIDEO_MODE_FORMAT7_7"
};


// Set up a camera object
VidereStereoDriver::VidereStereoDriver(uint64_t guid, size_t bsize)
{
    // Initialize the dcam system
    initialize_camera(guid);

    bufferSize = bsize;
    camPolicy = DC1394_CAPTURE_POLICY_POLL;
    camFrame = NULL;

    isSTOC = false;
    isVidereStereo = false;
    isVidere = false;
    isColor = false;
    procMode = PROC_MODE_NONE;

    // set up stereo image data
    stIm = new StereoData();

    // Check Videre camera type and local params
    if (!strcmp(getModel(), "MDS-STH")) // Videre-type camera
    {
        isVidere = true;

        PRINTF("[dcam] Videre camera, getting local params");
        uint32_t qval;

        // firmware level
        qval = getRegister(VIDERE_LOCAL_PARAM_BASE + VIDERE_CAM_FW_LEVEL_OFFSET);
        int major = (qval & 0x0000ff00) >> 8;
        int minor = (qval & 0x000000ff);

        // check for local parameters
        if ((qval >> 16) != 0 || major < 2 || minor > 10)
        {
            PRINTF("[dcam] No local parameters");
        }
        else
        {
            // Camera and imager firmware
            camFirmware = qval & 0xffff;
            PRINTF("[dcam] Camera firmware: %02d.%02d", major, minor);

            qval = getRegister(VIDERE_LOCAL_PARAM_BASE + VIDERE_CAM_LEVEL_OFFSET);
            imFirmware = qval & 0xff;
            PRINTF("[dcam] Imager firmware: %04x", imFirmware);

            if ((qval & 0xff0000) == 0x080000)
            {
                isVidereStereo = true;
                PRINTF("[Dcam] Found stereo device");
            }

            // STOC
            qval = getRegister(VIDERE_LOCAL_PARAM_BASE + VIDERE_CAM_PROC_OFFSET);
            stocFirmware = (qval & 0xffff00) >> 8;
            major = (stocFirmware & 0xff00) >> 8;
            minor = stocFirmware & 0xff;
            PRINTF("[dcam] STOC version: %02d.%02d", major, minor);

            if (major > 0 && major < 0xff && minor != 0xff) // check for odd firmware values
            {
                isSTOC = true;
                procMode = (videre_proc_mode_t) (qval & 0x000000ff);

                // this sets the Config bits on faulty FPGA firmware (version 4.1 and below)
                qval = 0x08000000 | (0x9C << 16);
                setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
            }

            // STOC thresholds
            qval = getRegister(VIDERE_LOCAL_PARAM_BASE + VIDERE_CAM_PROC_THRESH_OFFSET);
            PRINTF("[dcam] STOC thresholds: %08x", qval);

            // parameter string
            if (getParameters() != NULL) { PRINTF("[dcam] Calibration, %Zu bytes", strlen(stIm->params)); }
            else { PRINTF("[dcam] No calibration"); }
        }
    }

    // check for color/monochrome camera
    isColor = false;

    if (hasFeature(DC1394_FEATURE_WHITE_BALANCE))
    {
        isColor = true;
        PRINTF("[dcam] Color device");
    }
    else
    {
        PRINTF("[dcam] Monochrome device");
    }

    setRawType();

    // check registers
    uint32_t qval;
    qval = getRegister(0x404);
    PRINTF("[dcam] Feature register hi: %08x", qval);
    qval = getRegister(0x408);
    PRINTF("[dcam] Feature register lo: %08x", qval);

    // set up max/min values
    // NOTE: on Videre cameras with FW firmware < 6.0, control reg
    //   does not have presence switch on

    expMin = expMax = 0;
    gainMin = gainMax = 0;
    brightMin = brightMax = 0;

    if (isVidere && camFirmware < 0x0600)
    {
        expMax = 530;
        gainMax = 48;
        brightMax = 255;

        uint32_t qval;
        qval = getRegister(0x504); // exposure inquiry reg
        expMax = qval & 0x3ff;
        PRINTF("[Dcam] Exposure min/max: [%d,%d]", expMin, expMax);

        qval = getRegister(0x520); // exposure inquiry reg
        gainMax = qval & 0x3ff;
        PRINTF("[Dcam] Gain min/max: [%d,%d]", gainMin, gainMax);

        qval = getRegister(0x500); // brightness inquiry reg
        brightMax = qval & 0x3ff;
        PRINTF("[Dcam] Brightness min/max: [%d,%d]", brightMin, brightMax);
    }
    else
    {
        if (hasFeature(DC1394_FEATURE_EXPOSURE))
        {
            getFeatureBoundaries(DC1394_FEATURE_EXPOSURE, expMin, expMax);
            PRINTF("[Dcam] Exposure min/max: [%d,%d]", expMin, expMax);
        }
        else
        {
            PRINTF("[Dcam] No exposure feature");
        }

        if (hasFeature(DC1394_FEATURE_GAIN))
        {
            getFeatureBoundaries(DC1394_FEATURE_GAIN, gainMin, gainMax);
            PRINTF("[Dcam] Gain min/max: [%d,%d]", gainMin, gainMax);
        }
        else
        {
            PRINTF("[Dcam] No gain feature");
        }

        if (hasFeature(DC1394_FEATURE_BRIGHTNESS))
        {
            getFeatureBoundaries(DC1394_FEATURE_BRIGHTNESS, brightMin, brightMax);
            PRINTF("[Dcam] Brightness min/max: [%d,%d]", brightMin, brightMax);
        }
        else
        {
            PRINTF("[Dcam] No brightness feature");
        }

        if (isColor)
        {
            getFeatureBoundaries(DC1394_FEATURE_WHITE_BALANCE, whiteBalanceMin, whiteBalanceMax);
            PRINTF("[Dcam] Whitebalance min/max: [%d,%d]", whiteBalanceMin, whiteBalanceMax);
        }
        else
        {
            PRINTF("[Dcam] No whitebalnce feature");
        }
    }

    // set up a Videre stereo cam
    if (isVidereStereo)
    {
        char *params;
        params = getParameters();

        if (params == NULL) { PRINTF("Could not get parameters from camera\n"); }
        else { stIm->extractParams(params); }
    }
}

// Tear down camera object
VidereStereoDriver::~VidereStereoDriver()
{
    if (dcCam != NULL) { cleanup(); }
    delete stIm;
}

void VidereStereoDriver::initialize_camera(uint64_t req_guid)
{
    dcRef = dc1394_new();

    if (dcRef == NULL)
    {
        throw VidereStereoDriverException("Could not initialize dc1394_context.\n                      \
                             Make sure /dev/raw1394 exists and you have RW permissions\n \
                             and libraw1394-dev package is installed.");
    }

    dc1394camera_list_t* list;
    CHECK_ERR(dc1394_camera_enumerate(dcRef, &list), "Could not enumerate cameras");

    if (list->num > 0)
    {
        guid = list->ids[0].guid;
        dc1394camera_t* camera = dc1394_camera_new((dc1394_t*) dcRef, guid);
        dc1394_camera_free_list(list);

        if (!camera) { throw VidereStereoDriverException("Could not acquire camera to reset bus"); }

        PRINTF("Resetting bus");
        dc1394_reset_bus(camera);

        PRINTF("Initializing camera, turning off ISO");
        dc1394_video_set_transmission(camera, DC1394_OFF);

        dc1394_camera_free(camera);
        dc1394_free((dc1394_t*) dcRef);

        dcRef = dc1394_new();

        if (req_guid != 0) { dcCam = dc1394_camera_new(dcRef, req_guid); }
        else { dcCam = dc1394_camera_new(dcRef, guid); }

        if (!dcCam) { throw VidereStereoDriverException("Could not create camera"); }
        CHECK_ERR(dc1394_video_get_supported_modes(dcCam, &camModes), "Could not get supported modes");
    }

    if (dcRef == NULL)
    {
        throw VidereStereoDriverException("Could not initialize dc1394_context.\n                      \
                             Make sure /dev/raw1394 exists and you have RW permissions\n \
                             and libraw1394-dev package is installed.");
    }
}

void VidereStereoDriver::fini()
{
    if (dcRef != NULL) { dc1394_free((dc1394_t*) dcRef); }
}

// mode strings from mode
const char* VidereStereoDriver::getModeString(dc1394video_mode_t mode)
{
    if (mode < DC1394_VIDEO_MODE_MAX) { return modestrings[mode - DC1394_VIDEO_MODE_MIN]; }
    else { return ""; }
}

void VidereStereoDriver::cleanup()
{
    dc1394_video_set_transmission(dcCam, DC1394_OFF);
    dc1394_capture_stop(dcCam);
    dc1394_camera_free(dcCam);
    dcCam = NULL;
}

// Return a list of modes
dc1394video_modes_t* VidereStereoDriver::getModes()
{
    CHECK_READY();
    return &camModes;
}

// Model name
char* VidereStereoDriver::VidereStereoDriver::getModel()
{
    return dcCam->model;
}

// Vendor name
char* VidereStereoDriver::getVendor()
{
    return dcCam->vendor;
}

// Set up image format
void VidereStereoDriver::setFormat(dc1394video_mode_t video, dc1394framerate_t fps, dc1394speed_t speed)
{
    stIm->releaseBuffers();
    videoMode = video;

    // tear down any previous capture setup
    dc1394_capture_stop(dcCam);

    // check for valid video mode
    size_t i;
    for (i = 0; i < camModes.num; ++i)
    {
        if (camModes.modes[i] == video) { break; }
    }

    // oops, haven't found it
    if (i >= camModes.num)
    {
        char msg[256];
        snprintf(msg, 256, "setFormat: not a valid mode: %s", getModeString(video));
        throw VidereStereoDriverException(msg);
    }

    CHECK_ERR_CLEAN(dc1394_video_set_mode(dcCam, video), "Could not set video mode");
    CHECK_ERR_CLEAN(dc1394_video_set_iso_speed(dcCam, speed), "Could not set iso speed");
    CHECK_ERR_CLEAN(dc1394_video_set_framerate(dcCam, fps), "Could not set framerate");
    CHECK_ERR_CLEAN(dc1394_capture_setup(dcCam, bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT), "Could not setup camera.");

    setRawType();
}

// Start and stop streaming
void VidereStereoDriver::start()
{
    CHECK_READY();
    CHECK_ERR_CLEAN(dc1394_video_set_transmission(dcCam, DC1394_ON), "Could not start camera iso transmission");

    // Some camera take a little while to start.  We check 10 times over the course of a second:
    int tries = 10;

    while (tries-- > 0)
    {
        // now check if we have started transmission, no error from set_transmission
        dc1394switch_t pwr;
        dc1394_video_get_transmission(dcCam, &pwr);

        if (pwr == DC1394_ON)
        {
            started = true;
            setCompanding(true);  // must be set after starting camera
            setUniqueThresh(stIm->uniqueThresh);
            setTextureThresh(stIm->textureThresh);
            return;
        }

        usleep(10000);
    }

    throw VidereStereoDriverException("Camera iso transmission did not actually start.");
}

void VidereStereoDriver::stop()
{
    if (camFrame) { CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, camFrame), "Could not release frame"); }

    camFrame = NULL;
    CHECK_READY();
    CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_OFF), "Could not stop camera iso transmission");

    started = false;
}

// Getting images
// Waits for the next image available, up to ms for timeout
//   Assumes capture policy of POLL
// Stores the next available image into the class instance
bool VidereStereoDriver::getImage(int ms)
{
    CHECK_READY();

    if (!started) { return false; }

    // release previous frame, if it exists
    if (camFrame) { CHECK_ERR_CLEAN(dc1394_capture_enqueue(dcCam, camFrame), "Could not release frame"); }
    camFrame = NULL;

    // get the image
    while (1)
    {
        CHECK_ERR_CLEAN( dc1394_capture_dequeue(dcCam, camPolicy, &camFrame), "Could not capture frame");

        if (camFrame == NULL)
        {
            if (ms <= 0) { break; }
            ms -= 10;
            usleep(10000);
        }
        else
        {
            // break;
            while (1)   // flush the buffer, get latest one
            {
                dc1394video_frame_t* f = NULL;
                dc1394_capture_dequeue(dcCam, camPolicy, &f);

                if (f != NULL)
                {
                    dc1394_capture_enqueue(dcCam, camFrame);
                    camFrame = f;
                }
                else
                {
                    break;
                }
            }

            break;
        }
    }

    // transfer info
    if (camFrame)
    {
        uint8_t* imRaw = camFrame->image;

        // fill in timestamps
        ros::Time stamp = ros::Time(camFrame->timestamp * 1.e-6);
        stIm->left_info.header.stamp = stamp;
        stIm->left_raw.header.stamp = stamp;
        stIm->right_info.header.stamp = stamp;
        stIm->right_raw.header.stamp = stamp;
        stIm->img_disp.header.stamp = stamp;
        stIm->img_disp.image.header.stamp = stamp;

        stIm->setSize(camFrame->size[0], camFrame->size[1]);
        stIm->hasDisparity = false;

        stereo_msgs::DisparityImage& img_disparity = stIm->img_disp;

        // check for single-device stereo, and process
        if (isVidereStereo)
        {
            stIm->stereo_model.fromCameraInfo(stIm->left_info, stIm->right_info);

            uint8_t* img_left_data = NULL;
            uint8_t* img_right_data = NULL;

            size_t img_left_size;
            size_t img_right_size;

            switch (rawType)
            {
                case VIDERE_STOC_RECT_RECT:
                case VIDERE_STOC_RAW_RAW_MONO:
                case VIDERE_STEREO_MONO:
                    fillImageMsg(stIm->left_raw, sensor_msgs::image_encodings::MONO8, sizeof(uint8_t));
                    img_left_data = stIm->left_raw.data.data();
                    img_left_size = stIm->left_raw.data.capacity();

                    fillImageMsg(stIm->right_raw, sensor_msgs::image_encodings::MONO8, sizeof(uint8_t));
                    img_right_data = stIm->right_raw.data.data();
                    img_right_size = stIm->right_raw.data.capacity();

                    stereoDeinterlace(imRaw, &img_left_data, &img_left_size, &img_right_data, &img_right_size);
                    break;

                case VIDERE_STOC_RAW_RAW_GRBG:
                case VIDERE_STEREO_RGGB:
                case VIDERE_STEREO_BGGR:
                case VIDERE_STEREO_GRBG:
                    fillImageMsg(stIm->left_raw, sensor_msgs::image_encodings::BAYER_GRBG8, sizeof(uint8_t));
                    img_left_data = stIm->left_raw.data.data();
                    img_left_size = stIm->left_raw.data.capacity();

                    fillImageMsg(stIm->right_raw, sensor_msgs::image_encodings::BAYER_GRBG8, sizeof(uint8_t));
                    img_right_data = stIm->right_raw.data.data();
                    img_right_size = stIm->right_raw.data.capacity();

                    stereoDeinterlace(imRaw, &img_left_data, &img_left_size, &img_right_data, &img_right_size);
                    break;

                case VIDERE_STOC_RECT_DISP:
                case VIDERE_STOC_RAW_DISP_MONO:
                    fillImageMsg(stIm->left_raw, sensor_msgs::image_encodings::MONO8, sizeof(uint8_t));
                    img_left_data = stIm->left_raw.data.data();
                    img_left_size = stIm->left_raw.data.capacity();

                    stereoDeinterlace2(imRaw, &img_left_data, &img_left_size, &stIm->imDisp, &stIm->imDispSize);
                    stIm->hasDisparity = true;

                    fillDisparityMsg(img_disparity);
                    break;

                case VIDERE_STOC_RAW_DISP_GRBG:
                    fillImageMsg(stIm->left_raw, sensor_msgs::image_encodings::BAYER_GRBG8, sizeof(uint8_t));
                    img_left_data = stIm->left_raw.data.data();
                    img_left_size = stIm->left_raw.data.capacity();

                    stereoDeinterlace2(imRaw, &img_left_data, &img_left_size, &stIm->imDisp, &stIm->imDispSize);
                    stIm->hasDisparity = true;

                    fillDisparityMsg(img_disparity);
                    break;

                default:
                    break;
            }

            img_left_data = NULL;
            img_right_data = NULL;
        }
    }

    return (camFrame != NULL);
}

void VidereStereoDriver::setCapturePolicy(dc1394capture_policy_t p)
{
    camPolicy = p;
}

// Features
bool VidereStereoDriver::hasFeature(dc1394feature_t feature)
{
    CHECK_READY();
    dc1394bool_t present;
    CHECK_ERR_CLEAN(dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
    return (present == DC1394_TRUE);
}

void VidereStereoDriver::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  CHECK_READY();

    if (feature == DC1394_FEATURE_WHITE_BALANCE)
    {
        if (isVidere)
        {
            CHECK_ERR_CLEAN(dc1394_feature_whitebalance_set_value_blind(dcCam, value, value2), "Could not set feature");
        }
        else
        {
            CHECK_ERR_CLEAN(dc1394_feature_whitebalance_set_value(dcCam, value, value2), "Could not set feature");
        }
    }
    else
    {
        if (isVidere)
        {
            CHECK_ERR_CLEAN(dc1394_feature_set_value_blind(dcCam, feature, value), "Could not set feature");
        }
        else
        {
            CHECK_ERR_CLEAN(dc1394_feature_set_value(dcCam, feature, value), "Could not set feature");
        }
    }
}

void VidereStereoDriver::getFeatureBoundaries(dc1394feature_t feature, uint32_t& min, uint32_t& max)
{
    CHECK_READY();
    dc1394bool_t present;
    CHECK_ERR_CLEAN(dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");

    if (present == DC1394_TRUE)
    {
        CHECK_ERR_CLEAN(dc1394_feature_get_boundaries(dcCam, feature, &min, &max), "Could not find feature boundaries");
    }
}

void VidereStereoDriver::setFeatureAbsolute(dc1394feature_t feature, float value)
{
    CHECK_READY();
    dc1394bool_t present;
    CHECK_ERR_CLEAN(dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");

    if (present == DC1394_TRUE)
    {
        CHECK_ERR_CLEAN(dc1394_feature_set_absolute_control(dcCam, feature,  DC1394_ON), "Could not enable absolute control.");
        CHECK_ERR_CLEAN(dc1394_feature_set_absolute_value(dcCam, feature, value), "Could not set feature");
    }
}

void VidereStereoDriver::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
    CHECK_READY();

    if (isVidere)
    {
        CHECK_ERR_CLEAN(dc1394_feature_set_mode_blind(dcCam, feature, mode), "Could not set feature");
    }
    else
    {
        CHECK_ERR_CLEAN(dc1394_feature_set_mode(dcCam, feature, mode), "Could not set feature");
    }
}

void VidereStereoDriver::setRegister(uint64_t offset, uint32_t value)
{
    CHECK_READY();
    CHECK_ERR_CLEAN(dc1394_set_control_register(dcCam, offset, value), "Could not set control register");
}

uint32_t VidereStereoDriver::getRegister(uint64_t offset)
{
    CHECK_READY();
    uint32_t value;
    CHECK_ERR_CLEAN(dc1394_get_control_register(dcCam, offset, &value), "Could not get control register");
    return value;
}

// STOC modes
bool VidereStereoDriver::setProcMode(videre_proc_mode_t mode)
{
    CHECK_READY();
    if (!isSTOC) { return false; }

    procMode = mode;

    // set it while running
    uint32_t qval1 = 0x08000000 | (0x90 << 16) | ( ( mode & 0x7) << 16);
    uint32_t qval2 = 0x08000000 | (0x9C << 16);

    setRegister(0xFF000, qval1);
    setRegister(0xFF000, qval2);

    // set it while stopped
    uint32_t qval = (stocFirmware << 8) | procMode;
    setRegister(VIDERE_LOCAL_PARAM_BASE + VIDERE_CAM_PROC_OFFSET, qval);

    // set up image type
    setRawType();

    return true;
}

// Raw type
void VidereStereoDriver::setRawType()
{
    if (isSTOC)
    {
        switch (procMode)
        {
            case PROC_MODE_OFF:
            case PROC_MODE_NONE:
            case PROC_MODE_TEST:
                if (isColor) { rawType = VIDERE_STOC_RAW_RAW_GRBG; }
                else { rawType = VIDERE_STOC_RAW_RAW_MONO; }
                break;

            case PROC_MODE_RECTIFIED:
                rawType = VIDERE_STOC_RECT_RECT;
                break;

            case PROC_MODE_DISPARITY:
                rawType = VIDERE_STOC_RECT_DISP;
                break;

            case PROC_MODE_DISPARITY_RAW:
                if (isColor) { rawType = VIDERE_STOC_RAW_DISP_GRBG; }
                else { rawType = VIDERE_STOC_RECT_DISP; } // This is not what it SHOULD be, but what in fact comes out of the camera
                break;
        }
    }
    else if (isVidereStereo) // stereo device
    {
        // some videre cameras have camFrame as 0
        // imRaw = camFrame->image;
        if (isColor) { rawType = VIDERE_STEREO_GRBG; }
        else { rawType = VIDERE_STEREO_MONO; }
    }
    else
    {
        PRINTF("Setting type of video mode to %d", videoMode);

        switch (videoMode)
        {
            case DC1394_VIDEO_MODE_640x480_RGB8:
                rawType = COLOR_CODING_RGB8;
                break;
            case DC1394_VIDEO_MODE_640x480_MONO8:
                rawType = COLOR_CODING_MONO8;
                break;
            default:
                rawType = COLOR_CODING_MONO8;
        }
    }
}

// Parameters
char* VidereStereoDriver::retParameters()
{
    return stIm->params;
}

bool VidereStereoDriver::putParameters(char *bb)
{
    if (stIm->params) { delete [] stIm->params; }
    int n = strlen(bb);
    char* str = new char[n + 1];
    strcpy(str, bb);
    stIm->params = str;

    return true;
}

char* VidereStereoDriver::getParameters()
{
    if (stIm->params) { free(stIm->params); }

    uint32_t qval = getRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET);

    if (qval == 0xffffffff)
    {
        stIm->params = NULL;
    }
    else
    {
        char *buf = new char[4096 * 4];
        int n = 4096 * 4;
        char* bb = buf;

        // read in each byte
        int pos = 0;
        uint32_t quad;
        quad = getRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET + pos);

        while (quad != 0x0 && quad != 0xffffffff && n > 3)
        {
            int val;
            pos += 4;
            n -= 4;
            val = (quad >> 24) & 0xff;
            *bb++ = val;
            val = (quad >> 16) & 0xff;
            *bb++ = val;
            val = (quad >> 8) & 0xff;
            *bb++ = val;
            val = quad & 0xff;
            *bb++ = val;
            quad = getRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET + pos);
        }

        // just in case we missed the last zero
        *bb = 0;
        stIm->params = buf;
    }

    return stIm->params;
}

// set the calibration and image parameters on a camera
bool VidereStereoDriver::setParameters()
{
    if (!stIm->params) { return false; }

    PRINTF("%s", stIm->params);

    // check firmware version
    if (camFirmware < 0x0201)
    {
        PRINTF("Firmware version absent or too low");
        return false;
    }

    // erase any previous calibration
    uint32_t qval;
    qval = getRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET);
    //  PRINTF("[Dcam] Calibration start: 0x%08X", qval);

    if (qval == 0xffffffff)
    {
        PRINTF("No calibration parameters");
    }
    else
    {
        PRINTF("[Dcam] Erasing calibration parameters");
        for (unsigned int i = 0; i < VIDERE_CALIB_SIZE; i += 512)
        {
            setRegister(VIDERE_LOCAL_BASE, VIDERE_CAM_STORE_MAGIC_NUM);
            setRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET + i, ~VIDERE_CAM_STORE_MAGIC_NUM);
            usleep(100000);
        }
    }

    // write out each byte
    int pos = 0;
    uint32_t quad = 0;
    char *bb = stIm->params;
    int n = strlen(bb);
    PRINTF("[Dcam] Writing %d bytes", n);

    while (n--)
    {
        int b = *bb++;

        if (pos > VIDERE_CALIB_SIZE - 10)
        {
            PRINTF("[SetCalib] Calibration file too large");
            return false;
        }

        int n = pos % 4;
        int p = (3 - n) * 8;
        quad |= b << p;

        if (n == 3)
        {
            setRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET + pos - 3, quad);
            quad = 0;
        }

        pos++;
    }

    PRINTF("[SetCalib] Wrote %d bytes", pos);
    pos = pos - (pos % 4);
    setRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET + pos, quad);
    setRegister(VIDERE_LOCAL_BASE + VIDERE_CALIB_OFFSET + pos + 4, 0x0);

    return true;
}

// upload the parameters and firmware to a STOC device
// erases EEPROM first
bool VidereStereoDriver::setSTOCParams(uint8_t *cbuf, int cn, uint8_t *lbuf, int ln, uint8_t *rbuf, int rn)
{
    if (!isSTOC) { return false; }

    uint32_t qval;
    int v;

    // turn off FPGA
    qval = 0x0D100000;  // switch off FPGA
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
    qval = 0x0D120000;  // switch on EEPROM

    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
    PRINTF("[Device] Switched off FPGA; switched on EEPROM");

    // erase the flash
    PRINTF("[Device] Erasing flash");
    qval = 0x0D030000; // erase command
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);

    for (int i = 0; i < 20; ++i)
    {
        usleep(1000000);
        qval = 0x0F000000;
        setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
        usleep(10000);
        qval = getRegister(VIDERE_LOCAL_PARAM_BASE);
        v = qval & 0xff;
        PRINTF("[Device] Read: %02x", v);
        if (v == 0xff) { break; }
    }

    if (v != 0xff)
    {
        PRINTF("[Device] Couldn't erase flash!");
        return false;
    }

    PRINTF("[Device] Flash erased");

    // write and verify at addr 0
    bool success = store_eeprom_bytes(0, cbuf, cn);

    if (!success)
    {
        PRINTF("[Device] Failed on FPGA configuration");
        goto failconfig;
    }

    // check for warping
    if (ln == 0)    // no warping, return
    {
        PRINTF("[Device] No warp table, exiting");
        return true;
    }

    // now do left warp table
    PRINTF("[Device] Saving %d bytes to STOC", ln);
    success = store_eeprom_bytes(0x040000, lbuf, ln);

    if (!success)
    {
        PRINTF("[Device] Failed to save warp table to STOC");
        goto failwarp;
    }

    // right warp table
    PRINTF("[Device] Saving %d bytes to STOC", rn);
    success = store_eeprom_bytes(0x060000, rbuf, rn);

    if (!success)
    {
        PRINTF("[Device] Failed to save warp table to STOC");
        goto failwarp;
    }

    // restore FPGA operation
    failconfig:
    failwarp:
        qval = 0x0D130000;  // switch off EEPROM
        setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
        qval = 0x0D110000;  // switch on FPGA
        setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
        PRINTF("[Device] Re-configured FPGA");
        usleep(2000000);
        qval = 0x0D120000;  // switch on EEPROM
        setRegister(VIDERE_LOCAL_PARAM_BASE, qval);

    return true;
}

// companding and HDR
bool VidereStereoDriver::setCompanding(bool on)
{
    usleep(50000);

    if (on) { setRegister(0xFF000, 0x041C0003); }
    else { setRegister(0xFF000, 0x041C0002); }

    return true;
}

bool VidereStereoDriver::setHDR(bool on)
{
    usleep(50000);

    if (on) { setRegister(0xFF000, 0x040F0051); }
    else { setRegister(0xFF000, 0x040F0011); }

    return true;
}

//
// value boundaries are given by the max/min variables
//
void VidereStereoDriver::setExposure(int val, bool isauto)
{
    usleep(50000);

    uint32_t v;
    if (val < 0) { v = 0; }
    else { v = val; }

    if (v < expMin) { v = expMin; }
    if (v > expMax) { v = expMax; }

    if (isauto) { setFeatureMode(DC1394_FEATURE_EXPOSURE,DC1394_FEATURE_MODE_AUTO); }
    else { setFeature(DC1394_FEATURE_EXPOSURE, v); }      // ??? do we have to set manual here ???
}

void VidereStereoDriver::setGain(int val, bool isauto)
{
    usleep(50000);

    uint32_t v;
    if (val < 0) { v = 0; }
    else { v = val; }

    if (v < gainMin) { v = gainMin; }
    if (v > gainMax) { v = gainMax; }

    if (isauto) { setFeatureMode(DC1394_FEATURE_GAIN,DC1394_FEATURE_MODE_AUTO); }
    else { setFeature(DC1394_FEATURE_GAIN,v); }      // ??? do we have to set manual here ???
}

bool VidereStereoDriver::setMaxAutoVals(int exp, int gain)
{
    usleep(50000);

    uint32_t v;
    if (exp < 1) { exp = 1; }
    if (((uint32_t) exp) > expMax) { exp = expMax; }
    v = 0x04BD0000 | exp;
    setRegister(0xFF000, v);

    if (gain < 0) { gain = 0; }
    if (((uint32_t) gain) > gainMax) { gain = gainMax; }
    v = 0x04360000 | (gain+16);
    setRegister(0xFF000, v);

    return true;
}

// brightness
void VidereStereoDriver::setBrightness(int val, bool isauto)
{
    usleep(50000);

    uint32_t v;
    if (val < 0) { v = 0; }
    else { v = val; }

    if (v < brightMin) { v = brightMin; }
    if (v > brightMax) { v = brightMax; }

    if (isauto)
    {
        setFeatureMode(DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_AUTO);
    }
    else
    {
#if 0	// this doesn't seem to have any effect...
        if (v > 0)		// set the brightness target too
        {
            if (v > brightMax-6) v = brightMax-6;
            v = 0xff & v;
            v = 0x04460000 | v | ((v+6)<<8);
            PRINTF("setting v");
            v = 0x0446fff0;
            setRegister(0xFF000, v);
            usleep(50000);
            setRegister(0xFF000, 0x04478080);
        }
#endif
        setFeature(DC1394_FEATURE_BRIGHTNESS, v);   //   ??? do we have to set manual here ???
    }
}

void VidereStereoDriver::setWhiteBalance(int blue_val, int red_val, bool isauto)
{
    usleep(50000);

    uint32_t bv;
    uint32_t rv;

    if (blue_val < 0) { bv = 0; }
    else { bv = blue_val;}

    if (bv < whiteBalanceMin) { bv = whiteBalanceMin; }
    if (bv > whiteBalanceMax) { bv = whiteBalanceMax; }

    if (red_val < 0) { rv = 0; }
    else { rv = red_val;}

    if (rv < whiteBalanceMin) { rv = whiteBalanceMin; }
    if (rv > whiteBalanceMax) { rv = whiteBalanceMax; }

    if (isauto)
    {
        setFeatureMode(DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_AUTO);
    }
    else
    {
        setFeatureMode(DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_MANUAL);
        setFeature(DC1394_FEATURE_WHITE_BALANCE, blue_val, red_val);
    }
}

//
// upload bytes to on-camera EEPROM
//
volatile int xx = 0x1a2b3c4d;

bool VidereStereoDriver::store_eeprom_bytes(int addr, uint8_t *buf, int count)
{
    unsigned long qval;
    int addrhigh = addr & 0x00ff0000;
    int addrlow  = (addr & 0x0000ffff) << 8;
    int progress;
    int totprog = count / (200 * 16);
    unsigned char* cptr = buf;
    int v;

    PRINTF("[Device] Writing %d bytes to address %06x", count, addr);

    setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0D120000); // turn on EEPROM

    // set up addr
    qval = 0x09000000 | addrhigh;
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
    qval = 0x0A000000 | addrlow;
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);

    // write out bytes
    progress = 0;

    for (int i = count; i > 0; i -= 16)
    {
        int b0, b1;

        if (i < 16)
        {
            // last bytes
            while (i > 1)
            {
                // get next 2 bytes
                b0 = *cptr++;
                b1 = *cptr++;
                // write them out and save them in buffer for verify
                usleep(1000);
                qval = 0x0B000000 | (b0 << 16) | (b1 << 8);
                setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
                i -= 2;
            }

            break;
        }

        for (int j = 0; j < 8; ++j)
        {
            // get next 2 bytes
            b0 = *cptr++;
            b1 = *cptr++;
            // write them out and save them in buffer for verify
            qval = 0x0C000000 | (b0 << 16) | (b1 << 8);
            setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
        }

        // check that we've completed
        bool success = false;

        for (int j = 0; j < 5; ++j)
        {
            qval = getRegister(VIDERE_LOCAL_PARAM_BASE);
            v = qval & 0xff000000;

            if (v == 0)
            {
                v = qval & 0x00ff0000;
                if (v == 0) { success = true; }
                else { success = false; }
                break;
            }

            usleep(1000);
        }

        if (!success)
        {
            PRINTF("[Device] Failed to complete flash write");
            setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0D130000); // turn off EEPROM
            return false;
        }

        ++progress;

        if ((progress % 200) == 0)
        {
            PRINTF("[Device] Count %d of %d", progress / 200, totprog);
        }
    }

    PRINTF("[Device] Finished storing at address %06x, verifying...", addr+count/2);

    // Now do a verify
    usleep(10000);

#ifdef VERIFY_EEPROM
    // reset the read address
    qval = 0x09000000 | addrhigh;
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
    usleep(10000);
    qval = 0x0A000000 | addrlow;
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
    usleep(10000);

    // set up read addr in eeprom
    qval = 0x0E000000;
    setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
    usleep(10000);

    // ok, now read each 2 bytes, compare them
    cptr = buf;
    int errcnt = 0;

    for (int i = 0; i < count; i += 2)
    {
        // do a read
        setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0F000000);
        // get result
        int pcount = 200;
        //      usleep(3000);

        for (int j = 0; j < 5; ++j)
        {
            usleep(pcount); // usleep doesn't work well here, need a
                            // good way to chew up a short amount of time
                            // need something like udelay()
            pcount += 500;
            qval = getRegister(VIDERE_LOCAL_PARAM_BASE);
            v = qval & 0xff000000;
            if (v == 0) { break; }
            // usleep(10000);
        }

        if (v != 0)
        {
            PRINTF("[Device] Time out on verify read: %d %x", xx, v);
            setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0D130000); // turn off EEPROM
            return false;
        }

        // done, get result
        v = (qval & 0xff00) >> 8;

        if (v != *cptr)
        {
            PRINTF("[Device] Wrote %02x, read %02x at addr 0x%06x (byte %d)", *cptr, v, i / 2 + addr, i + addr * 2);
            errcnt++;
            // return;
        }

        cptr++;
        v = qval & 0xff;

        if (v != *cptr)
        {
            PRINTF("[Device] Wrote %02x, read %02x at addr 0x%06x (%d)", *cptr, v, i / 2 + addr, i + 1 + addr * 2);
            errcnt++;
            // return;
        }

        cptr++;

        if (errcnt > 20)
        {
            PRINTF("[Device] Errcnt >20, failing verify %d", xx);
            setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0D130000); // turn off EEPROM
            return false;
        }

        if (i != 0 && (i % (200 * 16)) == 0) { PRINTF("[Device] Verified %d of %d", i / (200 * 16), totprog); }
    }

    PRINTF("[Device] Verified!");
#endif

    setRegister(VIDERE_LOCAL_PARAM_BASE, 0x0D130000); // turn off EEPROM

    return true;
}

//
// generate an incremental rectification table,
//   suitable for loading onto a STOC device
//

#define SUBPIX 64.0

int VidereStereoDriver::getIncRectTable(uint8_t *dest)
{
#if 0
  // pixskip is the number of bytes between pixels in the same image.
  // by default, pixels are 8-bit, non-interleaved
  int pixskip = 1;

  int width = sp->linelen;
  int height = sp->lines;

  // output file size
  int outx = width;
  int outy = height;

  // hold pixel increment bits
  int pixinc, pcnt;

  PRINTF("[Warp Table]  Input image size: %d %d", width, height);
  PRINTF("[Warp Table] Output image size: %d %d", outx, outy);

  int count = 0;		// count of bytes
  int i, j;
  float x, y, rnd;
  int ox, oy, odx, ody, dx, dy, hx, hy;
  //  int xmax = 0, ymax = 0, xmin = 0, ymin = 0;
  int px, py;
  double xmax = 0, ymax = 0;
  double ex,ey;

  for (i=0; i<outy; i++)
    {
      dx = dy = 0;
      hx = hy = 0;
      pixinc = 0;
      pcnt = 0;
      for (j=0; j<outx; j++)
	{
	  origAddr(&x, &y, (float)j, (float)i, sp, which);
	  if (x > 0) rnd = 0.5; else rnd = -0.5;
	  px = (int)(x*SUBPIX + rnd);
	  if (y > 0) rnd = 0.5; else rnd = -0.5;
	  py = (int)(y*SUBPIX + rnd);
	  if (j > 0)		// first data point
	    {
	      dx = px - ox;
	      dy = py - oy;
	      if (j > 1)
		{
		  // restrict to +-1

		  ex = x - ((double)(ox+odx))/SUBPIX;
		  if (ex > 0)
		    hx = 1;
		  else
		    hx = -1;

		  ey = y - ((double)(oy+ody))/SUBPIX;
		  if (ey > 0)
		    hy = 1;
		  else
		    hy = -1;

		  ex = ex - (double)hx/SUBPIX;
		  if (fabs(ex) > xmax) xmax = fabs(ex);
		  ey = ey - (double)hy/SUBPIX;
		  if (fabs(ey) > ymax) ymax = fabs(ey);

		  dx = odx + hx;
		  dy = ody + hy;

		  // accumulate pixel increment shift
		  pixinc = (pixinc << 2);
		  if (hx == 1)
		    pixinc = pixinc | 0x02;
		  if (hy == 1)
		    pixinc = pixinc | 0x01;
		  // check for writing out
		  pcnt++;
		  if (pcnt >= 4)
		    {
		      *dest++ = pixinc;
		      count++;
		      pcnt = 0;
		      pixinc = 0;
		    }
		}               // j > 1
	      else
		{		// j = 1
		  *dest++ = dx;
		  *dest++ = dy;
		  count += 2;
		}
	      odx = dx;
	      ody = dy;
	      ox = ox + dx;
	      oy = oy + dy;
	    }                   // j > 0
	  else
	    {			// j = 0, start of line
	      ox = px;
	      oy = py;
	      *dest++ = (px & 0xff00) >> 8;
	      *dest++ = px & 0xff;
	      *dest++ = (py & 0xff00) >> 8;
	      *dest++ = py & 0xff;
//	      PRINTF("%03d %04x %04x", i, px, py);
	      count += 4;
	    }
	  if (hx > 1 || hx < -1 || hy > 1 || hy < -1)
	    PRINTF("[Warp Table] Increment too large");

	} // end of loop over line pixels

      // check if the last pixinc gets written out
      if (pcnt > 0)
	{
	  while (pcnt++ < 4)
	    pixinc = pixinc << 2;
	  *dest++ = pixinc;
	  count++;
	}
    }

  PRINTF("[Warp Table] Max X change: %f", xmax);
  PRINTF("[Warp Table] Max Y change: %f", ymax);
  PRINTF("[Warp Table] Size is %d bytes", count);
  return count;
#endif
  return 0;
}

// de-interlace stereo data, reserving storage if necessary
void VidereStereoDriver::stereoDeinterlace(uint8_t *src, uint8_t **d1, size_t *s1, uint8_t **d2, size_t *s2)
{
    size_t size = stIm->imWidth * stIm->imHeight;

    // need to check alignment here...
    if (*s1 < size)
    {
        MEMFREE(*d1);
        *d1 = (uint8_t*) MEMALIGN(size);
        *s1 = size;
    }

    if (*s2 < size)
    {
        MEMFREE(*d2);
        *d2 = (uint8_t*) MEMALIGN(size);
        *s2 = size;
    }

    uint8_t *dd1 = *d1;
    uint8_t *dd2 = *d2;

    for (size_t i = 0; i < size; ++i)
    {
        *dd2++ = *src++;
        *dd1++ = *src++;
    }
}

// de-interlace stereo data, reserving storage if necessary
// second buffer is 16-bit disparity data
void VidereStereoDriver::stereoDeinterlace2(uint8_t *src, uint8_t **d1, size_t *s1, int16_t **d2, size_t *s2)
{
    int w = stIm->imWidth;
    int h = stIm->imHeight;
    size_t size = w * h;

    // need to check alignment here...
    if (*s1 < size)
    {
        MEMFREE(*d1);
        *d1 = (uint8_t*) MEMALIGN(size);
        *s1 = size;
    }

    if (*s2 < size * 2)
    {
        MEMFREE(*d2);
        *d2 = (int16_t*) MEMALIGN(size * 2);
        *s2 = size * 2;
    }

    uint8_t *dd1 = *d1;
    int16_t *dd2 = *d2;

    int dt = stIm->imDtop;
    int dl = stIm->imDleft;
    int dw = stIm->imDwidth;
    int dh = stIm->imDheight;

    /*
    * source rectangle
    * ================
    * (w-dwidth, h-dheight) => upper left
    * (dwidth, dheight)     => size
    *
    * dest rectangle
    * ==============
    * (dleft-6,dtop)        => upper left
    * (dwidth,dheight)      => size
    */

    dd2 += (dt * w + dl - 6) - ((h - dh) * w + w - dw);

    size = (h - dh) * w;

    for (size_t i = 0; i < size; ++i)
    {
        dd2++;
        src++;
        *dd1++ = *src++;
    }

    size = dh * w;

    for (size_t i = 0; i < size; ++i)
    {
        *dd2++ = ((uint16_t) *src++) << 2;
        *dd1++ = *src++;
    }
}

void VidereStereoDriver::fillImageMsg(sensor_msgs::Image& img, std::string enc, uint32_t bpp)
{
    img.width = stIm->imWidth;
    img.height = stIm->imHeight;
    img.step = img.width * bpp;
    img.encoding = enc;
    img.data.resize(img.height * img.step);
}

void VidereStereoDriver::fillDisparityMsg(stereo_msgs::DisparityImage& img)
{
    static const double inv_dpp = 1.0 / stIm->dpp;

    // stereo parameters
    img.f = stIm->stereo_model.right().fx();
    img.T = stIm->stereo_model.baseline();

    // window of (potentially) valid disparities
    img.valid_window.x_offset = stIm->imDleft;
    img.valid_window.y_offset = stIm->imDtop;
    img.valid_window.width = stIm->imDwidth;
    img.valid_window.height = stIm->imDheight;

    // Disparity search range
    img.min_disparity = 0; // 0 - 63 in Videre STOC
    img.max_disparity = stIm->numDisp - 1;
    img.delta_d = inv_dpp;

    // Fill in DisparityImage image data, converting to 32-bit float
    sensor_msgs::Image& dimage = img.image;
    fillImageMsg(dimage, sensor_msgs::image_encodings::TYPE_32FC1, sizeof(float));

    cv::Mat_<int16_t> disparity16(stIm->imHeight, stIm->imWidth, (int16_t*) &stIm->imDisp[0], stIm->imWidth * sizeof(int16_t));

    // do speckle filtering
    cv::filterSpeckles(disparity16, 0, stIm->speckleRegionSize, stIm->speckleDiff, stIm->buffer);

    cv::Mat_<float> dmat(dimage.height, dimage.width, (float*) &dimage.data[0], dimage.step);

    // We convert from fixed-point to float disparity and also adjust for any x-offset between
    // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
    disparity16.convertTo(dmat, dmat.type(), img.delta_d, -(stIm->stereo_model.left().cx() - stIm->stereo_model.right().cx()));
    ROS_ASSERT(dmat.data == &dimage.data[0]);
}

bool VidereStereoDriver::setTextureThresh(int thresh)
{
    stIm->setTextureThresh(thresh);

    if (isSTOC)
    {
        thresh = thresh/3;
        usleep(50000);

        if (thresh < 0) { thresh = 0; }
        if (thresh > 63) { thresh = 63; }

        uint32_t t_thresh = 0x08000000 | (0x40 << 16) | ( thresh << 16);
        setRegister(0xFF000, t_thresh);
    }

    return true;
}

bool VidereStereoDriver::setUniqueThresh(int thresh)
{
    stIm->setUniqueThresh(thresh);

    if (isSTOC)
    {
        thresh = thresh/3;
        usleep(50000);

        if (thresh < 0) { thresh = 0; }
        if (thresh > 63) { thresh = 63; }

        uint32_t u_thresh = 0x08000000 | (0x00 << 16) | ( thresh << 16);
        setRegister(0xFF000, u_thresh);
    }

    return true;
}

bool VidereStereoDriver::setHoropter(int val)
{
    stIm->setHoropter(val);
    stIm->setDispOffsets(); // reset offsets

    // set it on STOC
    if (isSTOC)
    {
        usleep(50000);

        if (val < 0) { val = 0; }
        if (val > 63) { val = 63; }

        uint32_t u_val = 0x08000000 | (0xC0 << 16) | ( val << 16);
        setRegister(0xFF000, u_val);
    }

    return true;
}

bool VidereStereoDriver::setSpeckleSize(int val)
{
    stIm->speckleRegionSize = val;
    return true;
}

bool VidereStereoDriver::setSpeckleDiff(int val)
{
    stIm->speckleDiff = val;
    return true;
}
