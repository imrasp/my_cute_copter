/*
 * coordinate_converter.h
 *
 *  Created on: Apr 12, 2017
 *      Author: rasp
 */

#ifndef COORDINATE_CONVERTER_H_
#define COORDINATE_CONVERTER_H_


#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define EARTH_RADIUS 6378137
class Coordinate_Converter
{

public:

	float ori_gps[3];
	float ori_imu[3];

	Coordinate_Converter();
	Coordinate_Converter(float imu[3], float gps[3]);

	Mat LLAtoXYZ(float gps_pos[3]);


};


#endif /* COORDINATE_CONVERTOR_H_ */
