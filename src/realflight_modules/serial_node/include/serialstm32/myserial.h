#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

#include <sensor_msgs/Imu.h>
#include "serial_node/my_msg.h"
#include "serial_node/rawpose.h"

#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_attitude_quaternion.h"
#include "mavlink/mavlink_types.h"


#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
#define buf_size 128
//char totalbuf[buf_size];

ros::Publisher read_pub;
ros::Publisher rawpose_pub;

ros::Subscriber write_pub;
typedef union
{
    float data;
    unsigned char data8[4];
} IMU_data;

// raw
serial_node::my_msg ros_msg;  //接收到的数据
serial_node::rawpose raw_pose;
// ros
sensor_msgs::Imu imu_msg;

int initSerial();
void processFromSerial();
int mavlinkCharParse(uint8_t value);
void sendCallback(const serial_node::my_msg msg);

