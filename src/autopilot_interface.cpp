/*
 * autopilot_interface.cpp
 *
 *  Created on: Apr 27, 2017
 *      Author: rasp
 */

#include "autopilot_interface.h"

// ----------------------------------------------------------------------------------
//   Time
// ----------------------------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

void
set_position(float lat, float lon, float alt, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_POSITION;

	sp.coordinate_frame = MAV_FRAME_GLOBAL_INT;

	sp.lat_int = lat;
	sp.lon_int = lon;
	sp.alt = alt;

	printf("POSITION SETPOINT LLA = [ %.4f , %.4f , %.4f ] \n", sp.lat_int, sp.lon_int, sp.alt);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask =
			MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY;

	sp.coordinate_frame = MAV_FRAME_GLOBAL_INT;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

void
set_yaw(float yaw, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_YAW_ANGLE ;

	sp.yaw  = yaw;

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}

void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask &=
			MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_, char filename[80])
{
	//printf("Filename : %s \n", filename);
	//freopen (filename,"r+",plog);
	//freopen (filename,"r+",stdout);
	//printf("reopen log file...\n");

	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	takeoff_status = 0;
	land_status = 0;
	arm_status = 0;
	home_status = 0;
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

	init_coord_conversion = 0;

	ConvertCoordinate = Coordinate_Converter();

	imu_status = UNINITIAL_IMU;


}

Autopilot_Interface::
~Autopilot_Interface()
{
	printf("File closed \n");
	fclose(stdout);
}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint_type = LOCAL_SETPOINT;
	current_setpoint = setpoint;
}

void Autopilot_Interface::update_setpoint(mavlink_set_position_target_global_int_t setpoint)
{
	current_setpoint_type = GLOBAL_SETPOINT;
	current_setpoint_global = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	printf("read_messages()...\n");
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			//printf("message id : %u \n", message.msgid);
			//fprintf(plog,"%u\n",message.msgid);

			// Handle Message ID
			switch (message.msgid)
			{

			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
				mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
				current_messages.time_stamps.heartbeat = get_time_usec();
				this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
				break;
			}

			case MAVLINK_MSG_ID_SYS_STATUS:
			{
				//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
				mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
				current_messages.time_stamps.sys_status = get_time_usec();
				this_timestamps.sys_status = current_messages.time_stamps.sys_status;
				break;
			}

			case MAVLINK_MSG_ID_BATTERY_STATUS:
			{
				//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
				mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
				current_messages.time_stamps.battery_status = get_time_usec();
				this_timestamps.battery_status = current_messages.time_stamps.battery_status;
				break;
			}

			case MAVLINK_MSG_ID_RADIO_STATUS:
			{
				//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
				mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
				current_messages.time_stamps.radio_status = get_time_usec();
				this_timestamps.radio_status = current_messages.time_stamps.radio_status;
				break;
			}

			case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			{
				printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED \n");
				mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
				current_messages.time_stamps.local_position_ned = get_time_usec();
				this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;

				printf("LOCAL_POSITION_NED : %f %f %f \n", current_messages.local_position_ned.x,current_messages.local_position_ned.y,current_messages.local_position_ned.z);

				if (imu_status == UNINITIAL_IMU)
				{
					imu_status = INITIAL_IMU;
				}

				break;

			}

			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{
				printf("\n MAVLINK_MSG_ID_GLOBAL_POSITION_INT \n");
				mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
				current_messages.time_stamps.global_position_int = get_time_usec();
				this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;

				printf("GLOBAL_POSITION_INT : %f %f %f \n", (float)current_messages.global_position_int.lat, (float)current_messages.global_position_int.lon, (float)current_messages.global_position_int.alt);
				float gps[3] = {(float)current_messages.global_position_int.lat,(float)current_messages.global_position_int.lon,(float)current_messages.global_position_int.alt};

				// set initial params for coordinate conversion
				if(init_coord_conversion == false && imu_status == INITIAL_IMU )
				{
					printf("SET INITIAL COORDINATE CONVERSION... \n");
					float imu[3] = {current_messages.local_position_ned.x,current_messages.local_position_ned.y,current_messages.local_position_ned.z};

					ConvertCoordinate = Coordinate_Converter(imu, gps);
					init_coord_conversion = true;
				}
				else
				{
					//Mat result = ConvertCoordinate.gpstoimu(gps);
					Mat result = ConvertCoordinate.LLAtoXYZ(gps);
					printf("conversion : result : %f, %f, %f \n", result.at<float>(0), result.at<float>(1), result.at<float>(2));
				}

				break;
			}

			case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
			{
				//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
				mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
				current_messages.time_stamps.position_target_local_ned = get_time_usec();
				this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
				break;
			}

			case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
			{
				//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
				mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
				current_messages.time_stamps.position_target_global_int = get_time_usec();
				this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
				break;
			}

			case MAVLINK_MSG_ID_HIGHRES_IMU:
			{
				//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
				mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
				current_messages.time_stamps.highres_imu = get_time_usec();
				this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
				break;
			}

			case MAVLINK_MSG_ID_ATTITUDE:
			{
				//printf("MAVLINK_MSG_ID_ATTITUDE\n");
				mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
				current_messages.time_stamps.attitude = get_time_usec();
				this_timestamps.attitude = current_messages.time_stamps.attitude;
				break;
			}

			case MAVLINK_MSG_ID_HOME_POSITION:
			{
				//printf("MAVLINK_MSG_ID_HOME_POSITION\n");
				mavlink_msg_home_position_decode(&message, &(current_messages.home_position));
				current_messages.time_stamps.home_position = get_time_usec();
				this_timestamps.home_position = current_messages.time_stamps.home_position;
				break;
			}

			//case MAVLINK_MSG_ID_GPS_INPUT:
			case 232:
			{
				//printf("MAVLINK_MSG_ID_GPS_INPUT\n");
				mavlink_msg_gps_input_decode(&message, &(current_messages.gps_input));
				current_messages.time_stamps.gps_input = get_time_usec();
				this_timestamps.gps_input = current_messages.time_stamps.gps_input;


				printf("\n GPS_INPUT :: number of satellite : %u, HDOP : %.1f \n ", current_messages.gps_input.satellites_visible, current_messages.gps_input.hdop);

				break;
			}

			case MAVLINK_MSG_ID_GPS_STATUS:
			{
				//printf("MAVLINK_MSG_ID_GPS_STATUS\n");
				mavlink_msg_gps_status_decode(&message, &(current_messages.gps_status));
				current_messages.time_stamps.gps_status = get_time_usec();
				this_timestamps.gps_status = current_messages.time_stamps.gps_status;

				break;
			}

			case MAVLINK_MSG_ID_GPS_RAW_INT:
			{
				//printf("MAVLINK_MSG_ID_GPS_STATUS\n");
				mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw_int));
				current_messages.time_stamps.gps_raw_int = get_time_usec();
				this_timestamps.gps_raw_int = current_messages.time_stamps.gps_raw_int;

				printf("\n GPS_INPUT_RAW_INT :: number of satellite : %u, HDOP : %u \n ", current_messages.gps_raw_int.satellites_visible, current_messages.gps_raw_int.eph);

				break;
			}
			//ALTITUDE
			case MAVLINK_MSG_ID_ALTITUDE:
			{
				//printf("MAVLINK_MSG_ID_ALTITUDE\n");
				mavlink_msg_altitude_decode(&message, &(current_messages.altitude));
				current_messages.time_stamps.altitude = get_time_usec();
				this_timestamps.altitude = current_messages.time_stamps.altitude;

				//printf("Altitude : %f \n", current_messages.altitude.altitude_local);
				break;
			}

			default:
			{
				// printf("Warning, did not handle message id %i\n",message.msgid);
				break;
			}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
				//				this_timestamps.battery_status             &&
				//				this_timestamps.radio_status               &&
				//				this_timestamps.local_position_ned         &&
				//				this_timestamps.global_position_int        &&
				//				this_timestamps.position_target_local_ned  &&
				//				this_timestamps.position_target_global_int &&
				//				this_timestamps.highres_imu                &&
				//				this_timestamps.attitude                   &&
				this_timestamps.sys_status                 //&&
				//this_timestamps.gps_input                  &&
				//this_timestamps.gps_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	int len;

	// pull from position target
	if(current_setpoint_type == LOCAL_SETPOINT)
	{
		mavlink_set_position_target_local_ned_t sp = current_setpoint;

		// double check some system parameters
		if ( not sp.time_boot_ms )
			sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
		sp.target_system    = system_id;
		sp.target_component = autopilot_id;

		mavlink_message_t message;
		mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);

		len = write_message(message);
	}
	else
	{
		mavlink_set_position_target_global_int_t sp = current_setpoint_global;
		// double check some system parameters
		if ( not sp.time_boot_ms )
			sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
		sp.target_system    = system_id;
		sp.target_component = autopilot_id;

		mavlink_message_t message;
		mavlink_msg_set_position_target_global_int_encode(system_id, companion_id, &message, &sp);

		len = write_message(message);

	}

	// check the write
	if ( len <= 0 )
	{
		if(current_setpoint_type == LOCAL_SETPOINT)
			fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
		else
			fprintf(stderr,"WARNING: could not send POSITION_TARGET_GLOBAL_INT \n");
	}

	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	//com.command          = MAV_CMD_NAV_GUIDED_ENABLE | MAV_MODE_GUIDED_ARMED;

	// https://pixhawk.ethz.ch/mavlink/
	// MAV_MODE
	// These defines are predefined OR-combined mode flags.
	// There is no need to use values from this enum, but it simplifies the use of the mode flags.
	// Note that manual input is enabled in all modes as a safety override.

	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Start Arming
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
arm_control()
{
	// Should only send this command once
	if ( arm_status == false )
	{
		printf("ARMIMG...\n");

		// ----------------------------------------------------------------------
		//   TOGGLE ARMING STAGE
		// ----------------------------------------------------------------------

		// Sends the command to arm
		int success = toggle_arm_control( true );

		// Check the command was written
		if ( success )
			arm_status = true;
		else
		{
			fprintf(stderr,"Error: Unable to arm, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if arm
	else
	{
		printf("DISARMIMG...\n");

		// ----------------------------------------------------------------------
		//   TOGGLE ARMING STAGE
		// ----------------------------------------------------------------------

		// Sends the command to arm
		int success = toggle_arm_control( false );

		// Check the command was written
		if ( success )
			arm_status = false;
		else
		{
			fprintf(stderr,"Error: Unable to disarm, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");
	}


}


// ------------------------------------------------------------------------------
//   Set Home Position
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
set_home()
{

	// Should only send this command once
	if ( home_status == false )
	{
		printf("SETTING HOME POSITION...\n");

		// ----------------------------------------------------------------------
		//   TOGGLE SET HOME
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_set_home();

		// Check the command was written
		if ( success )
		{
			home_status = true;

			// CHECK HOME POSITION
			mavlink_home_position_t home = current_messages.home_position;
			printf("\n----------------------------------------------------------------\n"
					"1 Latitude (WGS84), in degrees, Longitude (WGS84, in degrees,  Altitude (AMSL), in meters * 1000 \n"
					"2 : this position in the local coordinate frame "
					"\n----------------------------------------------------------------\n");
			printf("1 CURRENT HOME POSITION - after set home (lat,lon,alt) = [ %f , %f, %f ] \n", (float)home.latitude, (float)home.longitude, (float)home.altitude);
			printf("2 CURRENT HOME POSITION - after set home (x,y,z) = [ %f , %f, %f ] \n", home.x, home.y, home.z);
		}
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Take off using local position
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
takeoff_local( float x, float y, float z, float speed )
//takeoff_local()
{

	// cannot sent takeoff command again until land command is activated
	if ( takeoff_status == false )
	{
		printf("TAKEOFF...\n");

		//int success = toggle_takeoff_local(float x, float y, float z, float speed);
		//int success = toggle_takeoff_local();

		//send takeoff command
		mavlink_command_long_t com;
		com.target_system    = system_id;
		com.target_component = autopilot_id;
		com.command          = MAV_CMD_NAV_TAKEOFF_LOCAL;
		com.confirmation = 0;
		com.param3 = speed; // Takeoff ascend rate [ms^-1]
		com.param5 = x; // X-axis position [m]
		com.param6 = y; // Y-axis position [m]
		com.param7 = z; // Z-axis position [m]

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		int len = serial_port->write_message(message);

		// Check the command was written
		if ( len )
		{
			printf("TAKEOFF USING LOCAL POSITION...\n");
			takeoff_status = true;
			land_status = false;
		}
		else
		{
			fprintf(stderr,"Error: unable to takeoff, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	}

}

// ------------------------------------------------------------------------------
//   Landing using local position
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
land_local( float x, float y, float z, float speed )
{

	// cannot sent land command again until takeoff command is activated
	if ( land_status == false )
	{
		printf("LAND...\n");

		//int success = toggle_takeoff_local(x, float y, float z, float speed);
		//int success = toggle_takeoff_local();

		//send takeoff command
		mavlink_command_long_t com;
		com.target_system    = system_id;
		com.target_component = autopilot_id;
		com.command          = MAV_CMD_NAV_LAND_LOCAL;
		com.confirmation = 0;
		com.param3 = speed; // Takeoff ascend rate [ms^-1]
		com.param5 = x; // X-axis position [m]
		com.param6 = y; // Y-axis position [m]
		com.param7 = z; // Z-axis position [m]

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		int len = serial_port->write_message(message);

		// Check the command was written
		if ( len )
		{
			printf("LAND USING LOCAL POSITION...\n");
			takeoff_status = false;
			land_status = true;
		}
		else
		{
			fprintf(stderr,"Error: unable to land, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	}

}

// ------------------------------------------------------------------------------
//   Take off using global position
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
takeoff_global( float lat, float lon, float alt )
//takeoff_local()
{

	// cannot sent takeoff command again until land command is activated
	if ( takeoff_status == false )
	{
		printf("TAKEOFF...\n");

		//send takeoff command
		mavlink_command_long_t com;
		com.target_system    = system_id;
		com.target_component = autopilot_id;
		com.command          = MAV_CMD_NAV_TAKEOFF;
		com.confirmation = 0;
		com.param5 = lat; // Latitude
		com.param6 = lon; // Longitude
		com.param7 = alt; // Altitude

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		int len = serial_port->write_message(message);

		// Check the command was written
		if ( len )
		{
			printf("TAKEOFF USING GLOBAL POSITION...\n");
			takeoff_status = true;
			land_status = false;
		}
		else
		{
			fprintf(stderr,"Error: unable to takeoff, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	}

}

// ------------------------------------------------------------------------------
//   Landing using global position
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
land_global( float lat, float lon, float alt )
{

	// cannot sent land command again until takeoff command is activated
	if ( land_status == false )
	{
		printf("LAND...\n");

		//send takeoff command
		mavlink_command_long_t com;
		com.target_system    = system_id;
		com.target_component = autopilot_id;
		com.command          = MAV_CMD_NAV_LAND;
		com.confirmation = 0;
		com.param5 = lat; // Latitude
		com.param6 = lon; // Longitude
		com.param7 = alt; // Altitude

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		int len = serial_port->write_message(message);

		// Check the command was written
		if ( len )
		{
			printf("LAND USING GLOBAL POSITION...\n");
			takeoff_status = false;
			land_status = true;
		}
		else
		{
			fprintf(stderr,"Error: unable to land, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	}

}

// ------------------------------------------------------------------------------
//   Landing using global position
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
RTL()
{

	// cannot sent RTL command again until takeoff command is activated
	if ( land_status == false )
	{
		printf("RETURNING TO LAUNCH...\n");

		//send takeoff command
		mavlink_command_long_t com;
		com.target_system    = system_id;
		com.target_component = autopilot_id;
		com.command          = MAV_CMD_NAV_RETURN_TO_LAUNCH;
		com.confirmation = 0;

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		int len = serial_port->write_message(message);

		// Check the command was written
		if ( len )
		{
			printf("RETURN TO LAUNCH...\n");
			takeoff_status = false;
			land_status = true;
		}
		else
		{
			fprintf(stderr,"Error: unable to return to launch, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	}
}

// ------------------------------------------------------------------------------
//   Toggle Arm State
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_arm_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	//com.command          = MAV_MODE_FLAG_SAFETY_ARMED;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation = 0;
	com.param1 = flag ? 1.0f : 0.0f;


	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Toggle Home setting
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_set_home()
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = 	MAV_CMD_DO_SET_HOME;
	com.confirmation = 0;
	com.param1 = 1;


	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Toggle Take off using local position
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_takeoff_local()
{

	mavlink_command_long_t com;
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_TAKEOFF_LOCAL;
	com.confirmation = 0;
	//com.param3 = speed; // Takeoff ascend rate [ms^-1]
	//com.param5 = x; // X-axis position [m]
	//com.param6 = y; // Y-axis position [m]
	//com.param7 = z; // Z-axis position [m]

	// get initial position
	mavlink_set_position_target_local_ned_t ip = initial_position;

	com.param3 = 3; // Takeoff ascend rate [ms^-1]
	com.param5 = ip.x; // X-axis position [m]
	com.param6 = ip.y; // Y-axis position [m]
	com.param7 = ip.z-5.0; // Z-axis position [m]


	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}
// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
			current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}


