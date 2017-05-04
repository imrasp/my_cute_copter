#include "coordinate_converter.h"

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Coordinate_Converter::
Coordinate_Converter()
{}

Coordinate_Converter::
Coordinate_Converter(float imu[3], float gps[3])
{
	//save origin
	ori_gps[0] = gps[0];
	ori_gps[1] = gps[1];
	ori_gps[2] = gps[2];

	//save origin
	ori_imu[0] = imu[0];
	ori_imu[1] = imu[1];
	ori_imu[2] = imu[2];

	printf("ori_imu = %f, %f, %f \n", imu[0], imu[1], imu[2]);
	printf("ori_gps = %f, %f, %f \n", gps[0], gps[1], gps[2]);
}

Mat
Coordinate_Converter::
LLAtoXYZ(float gps_pos[3])
{
	//  Latitude and Longitude, expressed as degrees * 1E7
	//  Altitude in meters, expressed as * 1000 (millimeters)

	float latrel = gps_pos[0] - ori_gps[0];
	float lonrel = gps_pos[1] - ori_gps[1];
	float altrel = gps_pos[2] - ori_gps[2];

	float x = 2 * EARTH_RADIUS * cos((ori_gps[0]+gps_pos[0]) / 2 / 1e+7 / 180*M_PI ) * sin(lonrel / 2 / 1e+7 / 180 * M_PI);
	float y = 2 * EARTH_RADIUS * sin(latrel / 2 / 1e+7 / 180* M_PI);
	float z = altrel / 1e+3;

	return (Mat_<float>(3,1) << x + ori_imu[0], y + ori_imu[1], z + ori_imu[2]);
}
