#include "serialstm32/myserial.h"
#include <iostream>
#include <string>
using namespace std;
serial::Serial my_serial;


int initSerial()
{
    try {
        my_serial.setPort("/dev/ttyUSB0");
        my_serial.setBaudrate(921600);
        serial::Timeout to=serial::Timeout::simpleTimeout(100);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch(serial::IOException& ioe)
    {
        ROS_INFO_STREAM("Unable to open serial port ");
        return 0;
    }
    if(my_serial.isOpen())
        ROS_INFO_STREAM("serial port is opened");
    else
        return 0;
    return 1;
};

void processFromSerial()
{
    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        int isparse = 0;
        unsigned char totalbuf[buf_size];


        size_t n=my_serial.available();
        //ROS_INFO_STREAM("n                    "<< int(n));
        my_serial.read(totalbuf,n);

        for(size_t i=0;i<n;i++) {
            isparse=mavlinkCharParse(totalbuf[i]);
            if (isparse == 1) {
                read_pub.publish(imu_msg);
            }
            else if(isparse == 2)  {
                rawpose_pub.publish(imu_msg);
                break;
            }
        }
        //ROS_INFO_STREAM("ros_msg.gyro_x  "<<ros_msg.gyro_x);
        ros::spinOnce();
        loop_rate.sleep();
    }
};

int mavlinkCharParse(uint8_t value)
{
    static mavlink_message_t RxMsg;
    static mavlink_status_t  RxStatus = {msg_received:0,buffer_overrun:0,parse_error:0,packet_idx:0,
                                         current_rx_seq:0,current_tx_seq:0,packet_rx_success_count:0,
                                         packet_rx_drop_count:0,
                                         parse_state:MAVLINK_PARSE_STATE_UNINIT};
    mavlink_message_t msg;
    mavlink_status_t  status;
    if (mavlink_frame_char_buffer(&RxMsg, &RxStatus, value, &msg, &status) == MAVLINK_FRAMING_OK) {
        switch(RxMsg.msgid){
            case MAVLINK_MSG_ID_SCALED_IMU:{
                mavlink_scaled_imu_t stm32_msg;
                mavlink_msg_scaled_imu_decode(&RxMsg,&stm32_msg);

                imu_msg.header.stamp=  ros::Time::now();
                imu_msg.header.frame_id = "base_link";

                imu_msg.angular_velocity.x = stm32_msg.xgyro/1000.0;
                imu_msg.angular_velocity.y = -stm32_msg.ygyro/1000.0;
                imu_msg.angular_velocity.z = -stm32_msg.zgyro/1000.0;
                imu_msg.linear_acceleration.x = stm32_msg.xacc/1000.0;
                imu_msg.linear_acceleration.y = -stm32_msg.yacc/1000.0;
                imu_msg.linear_acceleration.z = -stm32_msg.zacc/1000.0;

                imu_msg.orientation.x = 0;
                imu_msg.orientation.y = 0;
                imu_msg.orientation.z = 0;
                imu_msg.orientation.w = 0;
                //ROS_INFO_STREAM("1 succesful  ");
                return 1;
            }
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:{
                mavlink_attitude_quaternion_t maqt;
                mavlink_msg_attitude_quaternion_decode(&RxMsg,&maqt);

                imu_msg.header.frame_id = "base_link";
                imu_msg.header.stamp =  ros::Time::now();
                imu_msg.orientation.x = maqt.q1;
                imu_msg.orientation.y = maqt.q2;
                imu_msg.orientation.z = maqt.q3;
                imu_msg.orientation.w = maqt.q4;

                //ROS_INFO_STREAM("2 succesful  ");
                return 2;
            }

            default:
                break;
        }
    }
    return -1;
};

void sendCallback(const serial_node::my_msg msg)
{
    mavlink_message_t mmt;
    mavlink_scaled_imu_t to_stm32;

    to_stm32.time_boot_ms =  ros::Time::now().toSec();
    to_stm32.xgyro=msg.gyro_x*1000;
    to_stm32.ygyro=msg.gyro_y*1000;
    to_stm32.zgyro=msg.gyro_z*1000;
    to_stm32.xacc=msg.acc_x*1000;
    to_stm32.yacc=msg.acc_y*1000;
    to_stm32.zacc=msg.acc_z*1000;
    to_stm32.xmag = 0;
    to_stm32.ymag = 0;
    to_stm32.zmag = 0;
    ROS_INFO_STREAM("to_stm32.time_boot_ms  "<<to_stm32.time_boot_ms);
    ROS_INFO_STREAM("to_stm32.xgyro  "<<to_stm32.xgyro);
    ROS_INFO_STREAM("to_stm32.ygyro   "<<to_stm32.ygyro);
    ROS_INFO_STREAM("to_stm32.zgyro   "<<to_stm32.zgyro);
    ROS_INFO_STREAM("to_stm32.xacc  "<<to_stm32.xacc);
    ROS_INFO_STREAM("to_stm32.yacc  "<<to_stm32.yacc);
    ROS_INFO_STREAM("to_stm32.zacc  "<<to_stm32.zacc);

    //mavlink_msg_scaled_imu_encode(0, 0, &mmt, &to_stm32);
    //unsigned char send_buf[128];
    //unsigned char buf_len = mavlink_msg_to_send_buffer(send_buf, (const mavlink_message_t*)&mmt);
    //my_serial.write(send_buf,sizeof(send_buf));
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_uart_communication_pub");
    ros::NodeHandle nh("pub_imu_pose");
    read_pub = nh.advertise<sensor_msgs::Imu>("rawimu",1000);
    rawpose_pub = nh.advertise<sensor_msgs::Imu>("rawpose",1000);

    //write_pub=nh.subscribe("readimu",100,sendCallback);

    mavlink_channel_t mct;
    int isopen = initSerial();
    ROS_INFO_STREAM("isopen"<<isopen);
    if(isopen)
        processFromSerial();

}
