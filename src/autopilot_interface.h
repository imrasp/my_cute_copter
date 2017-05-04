/*
 * autopilot_interface.h
 *
 *  Created on: Apr 27, 2017
 *      Author: rasp
 */

#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include <common/mavlink.h>
#include "coordinate_converter.h"



// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

// bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111

#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_ACCELERATION 0b0000110000111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_YAW_RATE     0b0000010111111111


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// helper functions
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

void set_position(float lat, float lon, float alt, mavlink_set_position_target_global_int_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_global_int_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_global_int_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_global_int_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
	uint64_t home_position;
	uint64_t gps_input;
	uint64_t gps_status;
	uint64_t gps_raw_int;
	uint64_t altitude;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
		home_position =0;
		gps_input = 0;
		gps_status = 0;
		gps_raw_int = 0;
		altitude = 0;

	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// Home Position
	mavlink_home_position_t home_position;

	// GPS Input
	mavlink_gps_input_t gps_input;

	// GPS Status
	mavlink_gps_status_t gps_status;

	// GPS Raw INT
	mavlink_gps_raw_int_t gps_raw_int;

	// Altitude
	mavlink_altitude_t altitude;

	// System Parameters?


	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */
enum Imu_Status { UNINITIAL_IMU, INITIAL_IMU };
enum Current_Setpoint { LOCAL_SETPOINT, GLOBAL_SETPOINT };

class Autopilot_Interface
{


public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_,  char filename[80]);
	~Autopilot_Interface();

	char reading_status;
	char writing_status;
	char control_status;
	char arm_status;
	char home_status;
	char takeoff_status;
	char land_status;
	uint64_t write_count;

	bool init_imu_msg;

	Imu_Status imu_status;
	// -1, UNINITIZED_IMU : Unitialized
	// 1  : Updated
	//enum


	char init_coord_conversion;

	int system_id;
	int autopilot_id;
	int companion_id;

	//FILE* plog;
	Coordinate_Converter ConvertCoordinate;

	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void update_setpoint(mavlink_set_position_target_global_int_t setpoint);
	void read_messages();
	int  write_message(mavlink_message_t message);

	void enable_offboard_control();
	void disable_offboard_control();

	void set_home();

	void arm_control();

	void takeoff_local(float x,float y, float z, float speed);
	void land_local(float x,float y, float z, float speed);

	void takeoff_global( float lat, float lon, float alt );
	void land_global( float lat, float lon, float alt );
	void RTL();

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );


private:

	Serial_Port *serial_port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	Current_Setpoint current_setpoint_type;

	mavlink_set_position_target_local_ned_t current_setpoint;
	mavlink_set_position_target_global_int_t current_setpoint_global;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control( bool flag );

	int toggle_arm_control( bool flag );
	int toggle_set_home();
	int toggle_takeoff_local();

	void write_setpoint();

};


#endif /* AUTOPILOT_INTERFACE_H_ */
