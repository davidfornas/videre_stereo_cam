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

#include <sstream>
#include <iostream>

#include <videre_stereo_cam/stereoimage.h>

#define PRINTF(a...) printf(a)

StereoData::StereoData()
{
	// disparity buffer
	imDisp = NULL;
	imDispSize = 0;

	// nominal values
	imWidth = 640;
	imHeight = 480;

	left_info.distortion_model = "plumb_bob";
	right_info.distortion_model = "plumb_bob";

	left_info.D.resize(5);
	right_info.D.resize(5);

	corrSize = 15;
	filterSize = 11;
	dpp = 16;
	numDisp = 64;
	offx = 0;

	textureThresh = 10;
	uniqueThresh = 12;
	speckleDiff = 8;
	speckleRegionSize = 100;

	hasRectification = false;

	params = NULL;

	setDispOffsets();
}

StereoData::~StereoData()
{
	releaseBuffers();
}

bool StereoData::setHoropter(int val)
{
	if (val < 0) { val = 0; }
	if (val > 63) { val = 63; }
	offx = val;

	return true;
}

bool StereoData::setTextureThresh(int val)
{
	if (val < 0) { val = 0; }
	if (val > 100) { val = 10; }
	textureThresh = val;

	return true;
}

bool StereoData::setUniqueThresh(int val)
{
	if (val < 0) { val = 0; }
	if (val > 100) { val = 10; }
	uniqueThresh = val;

	return true;
}

void StereoData::setDispOffsets()
{
	/*
	 * disparity image size
	 * ====================
	 * dleft  : (logs + corrs - 2)/2 - 1 + offx
	 * dwidth : w - (logs + corrs + offx - 2)
	 * dtop   : (logs + corrs - 2)/2
	 * dheight: h - (logs + corrs)
	 *
	 */

	imDtop = (filterSize + corrSize - 2) / 2;
	imDleft = (filterSize + corrSize - 2) / 2 + (numDisp - 1 + offx);
	imDwidth = imWidth - (filterSize + corrSize - 2) - (numDisp - 1 + offx);
	imDheight = imHeight - (filterSize + corrSize);
}

void StereoData::releaseBuffers()
{
	MEMFREE(imDisp);
	imDisp = NULL;
	imDispSize = 0;
	hasDisparity = false;
}

// image size needs to deal with buffers
void StereoData::setSize(int width, int height)
{
	imWidth = width;
	imHeight = height;

	left_info.width = width;
	left_info.height = height;

	right_info.width = width;
	right_info.height = height;
}

bool StereoData::setSpeckleRegionSize(int val)
{
	speckleRegionSize = val;
	return true;
}

bool StereoData::setSpeckleDiff(int val)
{
	speckleDiff = val;
	return true;
}

//
// param sting parsing routines
//

#include <iostream>
#include <sensor_msgs/image_encodings.h>
using namespace std;

	template <class T>
void extract(std::string& data, std::string section, std::string param, T& t)
{
	size_t found = data.find(section);

	if (found != string::npos)
	{
		found = data.find(param,found);

		if (found != string::npos)
		{
			std::istringstream iss(data.substr(found+param.length()));
			iss >> t;
		}
	}
}

void extract(std::string& data, std::string section,
		std::string param, double *m, int n)
{
	size_t found = data.find(section);

	if (found != string::npos)
	{
		found = data.find(param,found);

		if (found != string::npos)
		{
			std::istringstream iss(data.substr(found + param.length()));
			double v;

			for (int i = 0; i < n; ++i)
			{
				iss >> v;
				m[i] = v;
			}
		}
	}
}

/* Best way to inject different parameters is when the driver is loading 
 * them from the camera memory. TO-DO: Implement set_camera_info service. */
void StereoData::parseCalibrationSVS(string params, stereo_side_t stereo_side, sensor_msgs::CameraInfo& cam_info)
{
	string side;
	//Calibration parameters obtained July 12th, 2014 using ROS' camera_calibration tool.

	//Left camera
	// K - original camera matrix
	double K1[9]={765.435631, 0.000000, 315.669691,
0.000000, 762.930964, 236.779297,
0.000000, 0.000000, 1.000000}; 
	// D - distortion params
	double D1[5]={-0.374507, 0.160579, 0.000960, -0.000006, 0.000000};
	// R - rectification matrix
	double R1[9]={ 0.999634, -0.002220, 0.026951,
0.002023, 0.999971, 0.007319,
-0.026966, -0.007262, 0.999610};
	// P - projection matrix
	double P1[12]={775.724937, 0.000000, 312.703789, 0.000000,
0.000000, 775.724937, 227.752966, 0.000000,
0.000000, 0.000000, 1.000000, 0.000000};
	//Right camera
	// K - original camera matrix
	double K2[9]={776.374248, 0.000000, 328.897337,
0.000000, 771.041454, 219.856622,
0.000000, 0.000000, 1.000000}; 
	// D - distortion params
	double D2[5]={-0.407322, 0.430714, -0.002940, 0.004867, 0.000000};
	// R - rectification matrix
	double R2[9]={ 0.999936, -0.011294, -0.000787,
0.011288, 0.999910, -0.007296,
0.000869, 0.007287, 0.999973};
	// P - projection matrix
	double P2[12]={775.724937, 0.000000, 312.703789, -68.472043,
0.000000, 775.724937, 227.752966, 0.000000,
0.000000, 0.000000, 1.000000, 0.000000};

	switch (stereo_side)
	{
		case SIDE_LEFT:
			side = "left";
			for(int i=0; i<9;i++){
				cam_info.K[i]=K1[i];
				cam_info.R[i]=R1[i];
			}
			for(int i=0; i<5;i++){
				cam_info.D[i]=D1[i];
			}
			for(int i=0; i<12;i++){
				cam_info.P[i]=P1[i];
			}
			break;

		case SIDE_RIGHT:    
			side = "right";
			for(int i=0; i<9;i++){
				cam_info.K[i]=K2[i];
				cam_info.R[i]=R2[i];
			}
			for(int i=0; i<5;i++){
				cam_info.D[i]=D2[i];
			}
			for(int i=0; i<12;i++){
				cam_info.P[i]=P2[i];
			}
			break;
	}

	//cam_info.P[3] *= .001;  // convert from mm to m
}

void StereoData::parseCalibrationOST(string params, stereo_side_t stereo_side, sensor_msgs::CameraInfo& cam_info)
{
	string side;
	switch (stereo_side)
	{
		case SIDE_LEFT:
			side = "left";
			break;
		case SIDE_RIGHT:
			side = "right";
			break;
	}

	// K - original camera matrix
	extract(params, "[" + side + " camera]", "camera matrix", &cam_info.K[0], 9);

	// D - distortion params
	extract(params, "[" + side + " camera]", "distortion", &cam_info.D[0], 5);

	// R - rectification matrix
	extract(params, "[" + side + " camera]", "rectification", &cam_info.R[0], 9);

	// P - projection matrix
	extract(params, "[" + side + " camera]", "projection", &cam_info.P[0], 12);
}

void StereoData::printCameraInfo(stereo_side_t stereo_side, const sensor_msgs::CameraInfo& cam_info)
{
	string side;

	switch (stereo_side)
	{
		case SIDE_LEFT:
			side = "left";
			break;
		case SIDE_RIGHT:
			side = "right";
			break;
	}

	PRINTF("[dcam] %s camera matrix (K)\n", side.c_str());

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			PRINTF(" %.4f", cam_info.K[i * 3 + j]);
		}

		PRINTF("\n");
	}

	PRINTF("\n[dcam] %s distortion vector (D)\n", side.c_str());

	for (int i = 0; i < 5; ++i)
	{
		PRINTF(" %.4f", cam_info.D[i]);
	}

	PRINTF("\n\n[dcam] %s rectification matrix (R)\n", side.c_str());

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			PRINTF(" %.4f", cam_info.R[i * 3 + j]);
		}

		PRINTF("\n");
	}

	PRINTF("\n[dcam] %s projection matrix (P)\n", side.c_str());

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			PRINTF(" %.4f", cam_info.P[i * 4 + j]);
		}

		PRINTF("\n");
	}

	PRINTF("\n");
}

void StereoData::printCalibration()
{
	PRINTF("[dcam] Disparity resolution: 1/%d pixel\n", dpp);
	PRINTF("[dcam] Correlation window: %d\n", corrSize);
	PRINTF("[dcam] Prefilter window: %d\n", filterSize);
	PRINTF("[dcam] Number of disparities: %d\n", numDisp);

	printCameraInfo(SIDE_LEFT, left_info);
	printCameraInfo(SIDE_RIGHT, right_info);

	if (hasRectification) { PRINTF("[dcam] Has rectification\n\n"); }
	else { PRINTF("[dcam] No rectification\n\n"); }

	PRINTF("\n[dcam] External translation vector\n");

	for (int i = 0; i < 3; ++i)
	{
		PRINTF(" %.4f", T[i]);
	}

	PRINTF("\n\n[dcam] External rotation vector\n");

	for (int i = 0; i < 3; ++i)
	{
		PRINTF(" %.4f", Om[i]);
	}

	PRINTF("\n\n");
}

//
// gets params from a string
// "SVS"-type parameter strings use mm for the projection matrices, convert to m
// "OST"-type parameter strings use m for projection matrices
//
void StereoData::extractParams(char *ps, bool store)
{
	std::string params;
	params = ps;

	if (store && ps != NULL)
	{
		if (this->params) { delete [] this->params; }
		char *bb = new char[strlen(ps)];
		strcpy(bb, ps);
		this->params = bb;
	}

	PRINTF("\n\n[extractParams] Parameters:\n\n");

	// Initialize Translation parameters
	for (int i = 0; i < 3; ++i)
	{
		T[i] = 0.0;
	}

	// Initialize Rotation parameters
	for (int i = 0; i < 3; ++i)
	{
		Om[i] = 0.0;
	}

	if (strncmp(ps, "# SVS", 5) == 0) // SVS-type parameters
	{
		PRINTF("[dcam] SVS-type parameters\n");

		// Left camera calibration parameters
		parseCalibrationSVS(params, SIDE_LEFT, left_info);

		// Right camera calibration parameters
		parseCalibrationSVS(params, SIDE_RIGHT, right_info);

		// external params of undistorted cameras
		extract(params, "[external]", "Tx", T[0]);
		extract(params, "[external]", "Ty", T[1]);
		extract(params, "[external]", "Tz", T[2]);
		extract(params, "[external]", "Rx", Om[0]);
		extract(params, "[external]", "Ry", Om[1]);
		extract(params, "[external]", "Rz", Om[2]);

		T[0] *= .001;
		T[1] *= .001;
		T[2] *= .001;
	}
	else // OST-type parameters
	{
		PRINTF("[dcam] OST-type parameters\n");

		// Left camera calibration parameters
		parseCalibrationOST(params, SIDE_LEFT, left_info);

		// Right camera calibration parameters
		parseCalibrationOST(params, SIDE_RIGHT, right_info);

		// external params of undistorted cameras
		extract(params, "[externals]", "translation", T, 3);
		extract(params, "[externals]", "rotation", Om, 3);
	}

	// disparity resolution
	extract(params, "[stereo]", "dpp", dpp);
	extract(params, "[stereo]", "corrxsize", corrSize);
	extract(params, "[stereo]", "convx", filterSize);
	extract(params, "[stereo]", "ndisp", numDisp);

	// check for left camera matrix
	if (left_info.K[0] == 0.0) { hasRectification = false; }
	else { hasRectification = true; }

	// check for right camera matrix
	if (right_info.K[0] == 0.0) { hasRectification = false; }

	printCalibration();
}

//
// Create parameters string and save it in the imLeft->params location
//
static int PrintMatStr(const double* mat, int n, int m, char* str)
{
	int c = 0;

	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			c += sprintf(&str[c], "%8.5f ", mat[i * m + j]);
		}

		c += sprintf(&str[c], "\n");
	}

	return c;
}

static void PrintMat(double* mat, int n, int m)
{
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			printf("%8.5f ", mat[i * m + j]);
		}

		printf("\n");
	}
}

static int PrintStr(int val, char* str)
{
	int c = 0;
	c += sprintf(&str[c], "%d ", val);

	return c;
}

char* StereoData::createParams(bool store)
{
	char* str = new char[4096];
	int n = 0;

	// header
	n += sprintf(str,"# oST version %d.%d parameters\n\n", OST_MAJORVERSION, OST_MINORVERSION);

	// stereo params
	n += sprintf(&str[n], "\n[stereo]\n");
	n += sprintf(&str[n], "\nndisp    ");
	n += PrintStr(numDisp, &str[n]);
	n += sprintf(&str[n], "\ndpp      ");
	n += PrintStr(dpp, &str[n]);
	n += sprintf(&str[n], "\ncorrsize ");
	n += PrintStr(corrSize, &str[n]);
	n += sprintf(&str[n], "\npresize  ");
	n += PrintStr(filterSize, &str[n]);

	// externals
	n += sprintf(&str[n], "\n\n[externals]\n");

	n += sprintf(&str[n], "\ntranslation\n");
	n += PrintMatStr(T, 1, 3, &str[n]);

	n += sprintf(&str[n], "\nrotation\n");
	n += PrintMatStr(Om, 1, 3, &str[n]);

	// left camera
	n += sprintf(&str[n], "\n[left camera]\n");

	n += sprintf(&str[n], "\ncamera matrix\n");
	n += PrintMatStr(left_info.K.data(), 3, 3, &str[n]);

	n += sprintf(&str[n], "\ndistortion\n");
	n += PrintMatStr(left_info.D.data(), 1, 5, &str[n]);

	n += sprintf(&str[n], "\nrectification\n");
	n += PrintMatStr(left_info.R.data(), 3, 3, &str[n]);

	n += sprintf(&str[n], "\nprojection\n");
	n += PrintMatStr(left_info.P.data(), 3, 4, &str[n]);

	// right camera
	n += sprintf(&str[n], "\n[right camera]\n");
	n += sprintf(&str[n], "\ncamera matrix\n");
	n += PrintMatStr(right_info.K.data(), 3, 3, &str[n]);

	n += sprintf(&str[n], "\ndistortion\n");
	n += PrintMatStr(right_info.D.data(), 1, 5, &str[n]);

	n += sprintf(&str[n], "\nrectification\n");
	n += PrintMatStr(right_info.R.data(), 3, 3, &str[n]);

	n += sprintf(&str[n], "\nprojection\n");
	n += PrintMatStr(right_info.P.data(), 3, 4, &str[n]);

	str[n] = 0; // just in case

	if (store)
	{
		if (params) { delete [] params; }
		char* bb = new char[n];
		strcpy(bb, str);
		params = bb;
	}

	return str;
}
