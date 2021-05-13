/*
Piksi Multi USB to ROS interface
Author: Max Henty-Scown

This program reads from a Swift navigation device connected to port PORT_NAME,
and converts messages to ros messages. Swift navigation device (Piksi Multi) Should be
configured to output only desired messages (529,526,258,2304) on desired port.

IMPORTANT. IMU raw readings need to be converted to m/s^2. IMU range is variable, should be set to
+-4g or Piksi multi setting 1. Similarly, gyro should be set to setting 1 (125deg/sec).

*/

#define PORT_NAME "/dev/ttyACM1"
#define IMU_RANGE 4 //gs
#define G_CONST 9.81
#define IMU_CONV (IMU_RANGE*G_CONST/32768)
#define GYRO_RANGE 125 //deg/s
#define DEG2RAD 0.01745329252
#define GYRO_CONV (GYRO_RANGE*DEG2RAD/32768)

//C headers
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

//ROS headers
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h> 

//Swift Binary Protocal (Protocal to communicate with Swift Piksi Multi)
extern "C"
{
#include "sbp.h" //Swift Binary Protocal common
#include "navigation.h" //SBP navigation messages
#include "imu.h" // SBP IMU messages
}
/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t sbp_state;

/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_cov_t      pos_llh_cov; //SBP message 529
msg_vel_ned_t      vel_ned; //SBP message 526
msg_gps_time_t     gps_time; //SBP message 258
msg_imu_raw_t      imu_raw; //SBP message 2304

/*
 * SBP callback nodes must be statically allocated. Each message ID / callback
 * pair must have a unique sbp_msg_callbacks_node_t associated with it.
 */
sbp_msg_callbacks_node_t pos_llh_cov_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t gps_time_node;
sbp_msg_callbacks_node_t imu_raw_node;

/*
* Because SBP callbacks are defined in lib, using global allocations for the ROS topics
* is very easy. This is a bad idea if this code is ever extended.
*/
ros::Publisher imu;
ros::Publisher navsat;
ros::Publisher navsat_vel;
  

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */
void sbp_pos_llh_cov_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh_cov = *(msg_pos_llh_cov_t *)msg;
  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header.frame_id = "Piksi_Antenna";
  fix_msg.header.stamp = ros::Time::now();
  fix_msg.latitude = pos_llh_cov.lat;
  fix_msg.longitude = pos_llh_cov.lon;
  fix_msg.altitude = pos_llh_cov.height;
  fix_msg.position_covariance_type = fix_msg.COVARIANCE_TYPE_KNOWN;
  fix_msg.position_covariance = {pos_llh_cov.cov_n_n,pos_llh_cov.cov_n_e,pos_llh_cov.cov_n_d,pos_llh_cov.cov_n_e,pos_llh_cov.cov_e_e,pos_llh_cov.cov_e_d,pos_llh_cov.cov_n_d,pos_llh_cov.cov_e_d,pos_llh_cov.cov_d_d};

  navsat.publish(fix_msg);

}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
  geometry_msgs::Vector3Stamped vel_msg;
  vel_msg.header.frame_id = "Piksi_Antenna";
  vel_msg.header.stamp = ros::Time::now();
  vel_msg.vector.x = vel_ned.n;
  vel_msg.vector.y = vel_ned.e;
  vel_msg.vector.z = vel_ned.d;

}
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}

/* Statically defines imu message, as most of the content is unchanging*/
sensor_msgs::Imu imu_msg;

void sbp_imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  //typecasts msg
  imu_raw = *(msg_imu_raw_t *)msg;

  // fills out the unchanging portion of the imu message
  // TODO measure the error & covariance. 
  imu_msg.angular_velocity_covariance = { 0,0,0,
                                          0,0,0,
                                          0,0,0};
  imu_msg.linear_acceleration_covariance = {0,0,0,
                                            0,0,0,
                                            0,0,0};
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.header.frame_id = "Piksi_IMU"; 

  //Changing portion of the message
  imu_msg.linear_acceleration.x = imu_raw.acc_x*IMU_CONV;
  imu_msg.linear_acceleration.y = imu_raw.acc_y*IMU_CONV;
  imu_msg.linear_acceleration.z = imu_raw.acc_z*IMU_CONV;
  imu_msg.angular_velocity.x = imu_raw.gyr_x*GYRO_CONV;
  imu_msg.angular_velocity.y = imu_raw.gyr_y*GYRO_CONV;
  imu_msg.angular_velocity.z = imu_raw.gyr_z*GYRO_CONV;
  imu_msg.header.stamp = ros::Time::now();

  imu.publish(imu_msg);
}


/*
 * Set up SwiftNav Binary Protocol (SBP) nodes; the sbp_process function will
 * search through these to find the callback for a particular message ID.
 *
 * Example: sbp_pos_llh_callback is registered with sbp_state, and is associated
 * with both a unique sbp_msg_callbacks_node_t and the message ID SBP_POS_LLH.
 * When a valid SBP message with the ID SBP_POS_LLH comes through the UART, written
 * to the FIFO, and then parsed by sbp_process, sbp_pos_llh_callback is called
 * with the data carried by that message.
 */


void sbp_setup(void)
{
  /* SBP parser state must be initialized before sbp_process is called. */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback,
                        NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_cov_callback,
                        NULL, &pos_llh_cov_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
                        NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_IMU_RAW, &sbp_imu_raw_callback,
                        NULL, &imu_raw_node);
}

int serial_port;

void serial_setup(void)
{
  serial_port = open(PORT_NAME, O_RDONLY);

   // Check for errors
   if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
      printf("Attempted open of port /dev/ttyACM1, Check Pkisi Connection\n");
   }

   struct termios tty;


   // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      exit(EXIT_FAILURE);
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      exit(EXIT_FAILURE);
  }
}

s32 serial_read(u8 *buff, u32 n, void *context)
{
  int i;
  i = read(serial_port, buff,n);
  return i;
}

int main(int argc, char **argv) {

  //setup ROS
  ros::init(argc, argv, "piksi");
  ros::NodeHandle n;
  //setup ROS publishers (format chosen to match hector_gazebo_plugins)
  //Using globals because its easy with how the SBP libs work, not good practice.
  imu = n.advertise<sensor_msgs::Imu>("imu", 50);                                                                       
  navsat = n.advertise<sensor_msgs::NavSatFix>("fix", 50);
  navsat_vel = n.advertise<geometry_msgs::Vector3Stamped>("fix_velocity", 50);
  
  //Sleep for a second, needed sometimes to allow ros publishers to start
  //ros::Duration(1).sleep(); 

  //setup serial connection with piksi
  serial_setup();

  //initilise SBP parser
  sbp_setup();

  
 
  while(true){
    sbp_process(&sbp_state, &serial_read);
  }

  close(serial_port);
  return 0; // success
}