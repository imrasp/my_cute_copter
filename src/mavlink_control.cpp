/*
 * mavlink_control.cpp
 *
 *  Created on: Apr 27, 2017
 *      Author: rasp
 */

#include "mavlink_control.h"
#include <stdio.h>
#include <string.h>

int
top (int argc, char **argv)
{

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);

	// --------------------------------------------------------------------------
	//   PREPARE FILE FOR LOG
	// --------------------------------------------------------------------------

	time_t t = time(0);   // get time now
	struct tm * now = localtime( & t );

	char filename[80];
	strftime(filename,80,"./log/LOG_%Y-%m-%d_%H:%M:%S.txt",now);
	plog = fopen(filename, "w");
	if(plog==NULL)
	{
		printf("error open file--plog : %d \n",plog);
		exit(-1);
	}
	//print to file without this line message will be printed to console
	//freopen (filename,"r+",stdout);

	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
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
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port, filename);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
	autopilot_interface.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface);

	// Give sometime for system to print out more information
	//sleep(10);

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------
	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// arm copter
	api.arm_control();
	sleep(5);
	int armed_state = api.current_messages.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
	printf("Armed? %s \n", armed_state ? "Yes" : "No");
	if (armed_state == false) return;

	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	if(api.current_messages.gps_raw_int.satellites_visible < 7)
	{
		// set current position to be home position
		api.set_home();
		sleep(5);

		// take off to 2 meters above the ground with speed 3 ms^-1
		api.takeoff_local(ip.x, ip.y, ip.z-2, 3.0);
		// hold position for 10 seconds
		sleep(10);
		mavlink_local_position_ned_t current_pos = api.current_messages.local_position_ned;
		printf("current position (NED):  %f %f %f (m)\n", current_pos.x, current_pos.y, current_pos.z );

		while(current_pos.z > -10.0)
		{ // if cannot get GPS and height less than 10, continue fly up and find gps
			if(api.current_messages.gps_raw_int.satellites_visible < 7)
			{
				mavlink_set_position_target_local_ned_t spl;
				current_pos = api.current_messages.local_position_ned;
				set_position( current_pos.x , current_pos.y, current_pos.z-2, spl);
				api.update_setpoint(spl);
				while (IsInWaypointLocal(api.current_messages.local_position_ned, spl, 0.2))
				{
					printf("current position (NED):  %f %f %f (m)\n", api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y, api.current_messages.local_position_ned.z );
					usleep(1000);
				}
				printf("current position after fly up 2m (NED):  %f %f %f (m)\n", current_pos.x, current_pos.y, current_pos.z );
			}
			else
			{
				mavlink_set_position_target_local_ned_t spl;
				current_pos = api.current_messages.local_position_ned;
				set_position( current_pos.x-3 , current_pos.y, current_pos.z, spl);
				api.update_setpoint(spl);
				while (IsInWaypointLocal(api.current_messages.local_position_ned, spl, 0.2))
				{
					printf("current position (NED):  %f %f %f (m)\n", api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y, api.current_messages.local_position_ned.z );
					usleep(1000);
				}
				printf("current position after fly 3m on x axis (NED):  %f %f %f (m)\n", current_pos.x, current_pos.y, current_pos.z );

				// hold position for 10 seconds
				sleep(10);

				mavlink_home_position_t home = api.current_messages.home_position;
				// land
				api.land_local(home.x, home.y, home.z, 2.0);
				usleep(100);

				break;
			}
		}
	}
	else // Get enough satellites
	{
		// set current position to be home position
		api.set_home();
		sleep(5);

		// take off to 3 meters above the ground with speed 3 ms^-1
		api.takeoff_global(ip.x, ip.y, ip.z-3);
		// hold position for 10 seconds
		sleep(10);

		mavlink_set_position_target_global_int_t spg;
		mavlink_global_position_int_t current_LLApos = api.current_messages.global_position_int;
		set_position( current_LLApos.lat-3 , current_LLApos.lon, current_LLApos.alt, spg);
		api.update_setpoint(spg);
		while (IsInWaypointGlobal(api.current_messages.global_position_int, spg, 0.2))
		{
			printf("current position (NED):  %f %f %f (m)\n", api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y, api.current_messages.local_position_ned.z );
			printf("current position (INT):  %f %f %f (m)\n", (float)api.current_messages.global_position_int.lat, (float)api.current_messages.global_position_int.lon, (float)api.current_messages.global_position_int.alt );
			usleep(1000);
		}
		printf("current position after fly 3m on latitude (INT):  %f %f %f (m)\n", (float)api.current_messages.global_position_int.lat, (float)api.current_messages.global_position_int.lon, (float)api.current_messages.global_position_int.alt );

		// hold position for 10 seconds
		sleep(10);

		// Return to launch
		api.RTL();
	}



	// disarm copter
	api.arm_control();
	usleep(100);
	sleep(5);

	api.disable_offboard_control();
	usleep(100);

	printf("File closed \n");
	fclose(plog);

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}

bool IsInWaypointLocal(mavlink_local_position_ned_t current, mavlink_set_position_target_local_ned_t goal, float radius)
{
	// Radios is in meters
	float dx = goal.x - current.x;
	float dy = goal.y - current.y;
	float dz = goal.z - current.z;
	float distance = sqrtf(pow(dx,2) + pow(dy,2) + pow(dz,2));

	if(distance < radius)
		return true;
	else
		return false;
}

bool IsInWaypointGlobal(mavlink_global_position_int_t current, mavlink_set_position_target_global_int_t goal, float radius)
{
	// Radios is in meters
	// suppose current GPS position is on ( 0, 0, 0 ) then find where is goal on XYZ coordinate,
	// then subtract them to get dx, dy, dz
	float a_ori [3] = {0.0, 0.0, 0.0};
	float a_current [3] = {(float)current.lat, (float)current.lon, (float)current.alt};
	float a_goal [3] = {(float)goal.lat_int, (float)goal.lon_int, (float)goal.alt};
	Coordinate_Converter cc = Coordinate_Converter();
	cc = Coordinate_Converter(a_ori, a_current);
	Mat goalXYZ = cc.LLAtoXYZ(a_goal);

	float dx = goalXYZ.at<float>(0) - 0.0;
	float dy = goalXYZ.at<float>(1) - 0.0;
	float dz = goalXYZ.at<float>(2) - 0.0;
	float distance = sqrtf(pow(dx,2) + pow(dy,2) + pow(dz,2));

	if(distance < radius)
		return true;
	else
		return false;
}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}





