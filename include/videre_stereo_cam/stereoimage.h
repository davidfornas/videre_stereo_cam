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

#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <malloc.h>
#include <stdint.h>

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

// version of parameter files
#define OST_MAJORVERSION 5
#define OST_MINORVERSION 0

#define MEMALIGN(x) memalign(16,x)
#define MEMFREE(x) {if (x) free(x);}

typedef enum
{
    SIDE_LEFT = 0,
    SIDE_RIGHT
} stereo_side_t;

typedef enum {
    COLOR_CODING_MONO8 = 3000,
    COLOR_CODING_MONO16,
    COLOR_CODING_BAYER8_RGGB,
    COLOR_CODING_BAYER8_BGGR,
    COLOR_CODING_BAYER8_GBRG,
    COLOR_CODING_BAYER8_GRBG,
    COLOR_CODING_BAYER16_RGGB,
    COLOR_CODING_BAYER16_BGGR,
    COLOR_CODING_BAYER16_GBRG,
    COLOR_CODING_BAYER16_GRBG,
    COLOR_CODING_RGB8,        // RGB order
    COLOR_CODING_RGBA8,       // RGBA order
    COLOR_CODING_RGB16,       // RGB order
    COLOR_CODING_RGBA16,      // RGBA order

    // these are stereo interlace encodings
    // Videre stereo:
    //   Mono has left/right pixels interlaced
    //   Color has left/right pixels interlace, bayer pixels
    //   STOC modes have rectified images, raw encodings, disparity, etc
    VIDERE_STEREO_MONO,
    VIDERE_STEREO_RGGB,
    VIDERE_STEREO_GRBG,
    VIDERE_STEREO_BGGR,
    VIDERE_STOC_RECT_RECT,    // left and right rectified mono
    VIDERE_STOC_RECT_DISP,    // left rectified mono, right disparity
    VIDERE_STOC_RAW_DISP_MONO,    // left raw mono, right disparity
    VIDERE_STOC_RAW_DISP_RGGB,    // left raw color, right disparity
    VIDERE_STOC_RAW_DISP_GRBG,    // left raw color, right disparity
    VIDERE_STOC_RAW_RAW_MONO, // left and right raw, mono
    VIDERE_STOC_RAW_RAW_RGGB, // left and right raw, color
    VIDERE_STOC_RAW_RAW_GRBG, // left and right raw, color

    COLOR_CODING_NONE     // no image info
} color_coding_t;

typedef enum {
    COLOR_CONVERSION_BILINEAR,
    COLOR_CONVERSION_EDGE
} color_conversion_t;


// stereo data structure
class StereoData
{
public:
    StereoData();
    ~StereoData();

    // image parameters
    int imWidth;
    int imHeight;
    void setSize(int width, int height); // sets individual image sizes too

    // left and right camera info
    sensor_msgs::CameraInfo left_info;
    sensor_msgs::CameraInfo right_info;

    // left and right image data
    sensor_msgs::Image left_raw;
    sensor_msgs::Image right_raw;

    image_geometry::StereoCameraModel stereo_model;
    stereo_msgs::DisparityImage img_disp;

    // rectification
    bool hasRectification;

    // disparity data
    int16_t *imDisp;    // disparity image, negative and zero are invalid pixels
    size_t imDispSize;  // size of image in bytes
    int dpp;            // disparity units per pixel, e.g., 16 is 1/16 pixel per disparity
    bool hasDisparity;  // true if disparity present
    int numDisp;        // number of search disparities, in pixels
    int offx;           // x offset of disparity search

    bool setHoropter(int offset);   // set horopter offset

    // valid stereo data rectangle
    int imDtop, imDleft;
    int imDwidth, imDheight;
    void setDispOffsets();  // reset them, based on stereo processing params

    cv::Mat buffer;         // buffer for speckle filtering

    // external parameters for undistorted images
    double T[3];    // pose of right camera in left camera coords
    double Om[3];   // rotation vector

    // buffers
    void releaseBuffers();  // get rid of all buffers

    // parameters
    void parseCalibrationSVS(std::string params, stereo_side_t side, sensor_msgs::CameraInfo& cam_info);
    void parseCalibrationOST(std::string params, stereo_side_t side, sensor_msgs::CameraInfo& cam_info);
    void printCameraInfo(stereo_side_t stereo_side, const sensor_msgs::CameraInfo& cam_info);
    void printCalibration();

    // extracts params from string and puts in vars, optionally stores into image object
    void extractParams(char *params, bool store = false);

    // takes parameters and puts them into a string, optionally stores into image object
    char *createParams(bool store = false);

    // stereo processing params
    int corrSize;   // correlation window size, assumed square
    int filterSize; // size of prefilter window, assumed square (0 if none)

    // filter thresholds
    int textureThresh;      // percent
    int uniqueThresh;       // percent
    int speckleDiff;        // max difference between adjacent disparities in a region
    int speckleRegionSize;  // minimum size of region to be not a speckle

    bool setTextureThresh(int thresh);
    bool setUniqueThresh(int thresh);
    bool setSpeckleDiff(int diff);
    bool setSpeckleRegionSize(int size);

    // raw parameter string
    char *params;   // on-camera parameters
};

#endif        // STEREOIMAGE_H
