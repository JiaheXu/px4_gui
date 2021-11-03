#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/px4_gui/qnode.hpp"

#include <tf/transform_broadcaster.h>
#include <QDebug>
#include "rosbag/bag.h"

#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>


//*****************************************************************************
//** Namespaces
//*****************************************************************************

namespace px4_gui {



QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{

}
QNode::~QNode() 
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }

    wait();
}
void QNode::stop() 
{
	ros::V_string v_nodes;
	ros::master::getNodes(v_nodes);

	std::string node_name = std::string("px4_gui");
	auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name.c_str());
	if (it != v_nodes.end()){
	    std::string cmd_str = "rosnode kill " + node_name;
	    int ret = system(cmd_str.c_str());
	}

}

// initialization *********************************
/*
bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"px4_gui");
    if ( ! ros::master::check() )
    {
        return false;
    }
}
*/
bool QNode::init()
{
    ros::init(init_argc,init_argv,"px4_gui");

    ros::start(); 
    ros::NodeHandle n;


    std::string string_var;
/*
* goal_pose_topic:          "/uav_offboard/GoalPose"
* velocity_yaw_topic:       "/uav_offboard/VelocityYaw"
* local_pose_topic:         "mavros/local_position/pose"
* state_topic:              "mavros/state"
* gripper_topic:            "/servo/command"
* position_setpoint_topic:  "mavros/setpoint_position/local"
* velocity_setpoint_topic:  "mavros/setpoint_velocity/cmd_vel"
*/
//////////////////////////////////////////////////////////////////
// rosparam can only be found after ros:start() !!!!!!!!!!!!!!!!!!
//////////////////////////////////////////////////////////////////    
//ROS_INFO("/goal_pose_topic: %s\n",string_var.c_str());

    ros::param::get("/goal_pose_topic", string_var);    
    goal_pose_topic = string_var.c_str();
    
    ros::param::get("/velocity_yaw_topic", string_var);
    velocity_yaw_topic = string_var.c_str();
    
    ros::param::get("/local_pose_topic", string_var);
    local_pose_topic = string_var.c_str();
    
    ros::param::get("/state_topic", string_var);
    state_topic = string_var.c_str();

    ros::param::get("/gripper_topic", string_var);
    gripper_topic = string_var.c_str();

    ros::param::get("/position_setpoint_topic", string_var);
    position_setpoint_topic = string_var.c_str();

    ros::param::get("/velocity_setpoint_topic", string_var);
    velocity_setpoint_topic = string_var.c_str();

    pose_topic= "mavros/local_position/pose";
    velocity_topic = "/mavros/local_position/velocity_body";
    accel_topic = "/mavros/imu/data";
    battery_topic = "mavros/battery";

    gripper_angle_topic = "/servo/angle";
    gripper_command_topic = "/servo/command";


    estimator_status_topic = "/mavros/estimator_status";
    companion_process_status_topic = "/mavros/companion_process/status";
    statustext_topic = "/mavros/statustext/recv";
    //ground_distance = "/mavros/px4flow/ground_distance";
    // battery subscriber
    power_sub = n.subscribe( battery_topic ,1,&QNode::batteryCallback,this);

    //position subscriber  ///////// might need to change to local_pose_topic
    pos_sub = n.subscribe(pose_topic,10,&QNode::poseCallback,this);

    //velocity subscriber
    velocity_sub = n.subscribe(velocity_topic,1,&QNode::velocityCallback,this);

    //acceleration subscriber
    accel_sub = n.subscribe(accel_topic,1,&QNode::accelCallback,this);

    //state subscriber
    state_sub = n.subscribe(state_topic,1,&QNode::stateCallback,this);

    //goal subscriber 
    goal_sub = n.subscribe(goal_pose_topic,1,&QNode::goalCallback,this);

/////

    position_setpoint_sub = n.subscribe(position_setpoint_topic,1,&QNode::position_setpointCallback,this);

    velocity_setpoint_sub = n.subscribe(velocity_setpoint_topic,1,&QNode::velocity_setpointCallback,this);

    velocity_yaw_sub = n.subscribe(velocity_yaw_topic,1,&QNode::velocity_yawCallback,this);

    gripper_sub = n.subscribe(gripper_topic,1,&QNode::gripperCallback,this);

    gripper_pub = n.advertise<std_msgs::UInt16>(gripper_topic, 1);

// error info monitor 
    estimator_status_sub = n.subscribe(estimator_status_topic,10,&QNode::estimator_statusCallback,this);
    companion_process_status_sub = n.subscribe(companion_process_status_topic,10,&QNode::companion_process_statusCallback,this);

//statustext_sub = n.subscribe(statustext_topic,1,&QNode::statustextCallback,this);

    return true;
}

void QNode::run() 
{
    if ( ros::ok() ) 
    {
	ros::spinOnce();
    }
}

void QNode::gripper_publish(int value)
{
qDebug() << "sent a gripper message !!!!!!!!!!!!!!!!!";

    std_msgs::UInt16 msg;
    msg.data = uint16_t(value);
    gripper_pub.publish(msg);

    ros::spinOnce();
}
void QNode::disinit()
{
    if(ros::isStarted())
    {
        ROS_INFO("ROS will shutdown");
        ros::shutdown();
        ros::waitForShutdown();
    }
    this->exit();
}

QMap<QString,QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString,QString> res;

    for(auto topic:topic_list)
    {
        res.insert(QString::fromStdString(topic.name),QString::fromStdString(topic.datatype));
    }

    return res;
}

void QNode::velocity_yawCallback(const aerial_autonomy::VelocityYaw::ConstPtr& Velocity_yaw)
{
        emit velocity_yaw( double(Velocity_yaw->vx) ,
                           double(Velocity_yaw->vy) ,
                           double(Velocity_yaw->vz) ,
                           double(Velocity_yaw->yaw) );
}

void QNode::gripperCallback(const std_msgs::UInt16 Command)
{
	emit gripper(int(Command.data));
}


//geometry_msgs/TwistStamped
void QNode::velocity_setpointCallback(const geometry_msgs::TwistStamped::ConstPtr& Velocity)
{
    emit velocity_setpoint( double(Velocity->twist.linear.x) , 
			    double(Velocity->twist.linear.y) ,
			    double(Velocity->twist.linear.z) ,
			    double(Velocity->twist.angular.x) , 
			    double(Velocity->twist.angular.y) ,
			    double(Velocity->twist.angular.z) );
}

void QNode::stateCallback(const mavros_msgs::State::ConstPtr& State)
{
//ROS_INFO("armed !!!!!!!!!!!!!!!!!!!!!! \n");
    emit state( State -> armed , State -> mode);
}


void QNode::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& Velocity)
{
    emit velocity( double(Velocity->twist.linear.x) , 
                   double(Velocity->twist.linear.y) ,
                   double(Velocity->twist.linear.z) ,
                   double(Velocity->twist.angular.x) , 
                   double(Velocity->twist.angular.y) ,
                   double(Velocity->twist.angular.z) );
}

void QNode::accelCallback(const sensor_msgs::Imu::ConstPtr& Accel)
{

    emit accel( double(Accel->linear_acceleration.x) ,
                double(Accel->linear_acceleration.y) , 
                double(Accel->linear_acceleration.z) );
}

void QNode::position_setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& Pos)
{
    double x,y,z;
    x = double( Pos->pose.position.x);
    y = double( Pos->pose.position.y);
    z = double( Pos->pose.position.z);

    tf::Quaternion q(
    Pos->pose.orientation.x,
    Pos->pose.orientation.y,
    Pos->pose.orientation.z,
    Pos->pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    emit position_setpoint(x,y,z,roll,pitch,yaw);

}
void QNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& Pos)
{
    double x,y,z;
    x = double( Pos->pose.position.x);
    y = double( Pos->pose.position.y);
    z = double( Pos->pose.position.z);

    tf::Quaternion q(
    Pos->pose.orientation.x,
    Pos->pose.orientation.y,
    Pos->pose.orientation.z,
    Pos->pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    emit position(x,y,z,roll,pitch,yaw);

}
void QNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& Pos)
{
//qDebug() << "a goal message !!!!!!!!!!!!!!!!!";
    double x,y,z;
    x = double( Pos->pose.position.x);
    y = double( Pos->pose.position.y);
    z = double( Pos->pose.position.z);

    tf::Quaternion q(
    Pos->pose.orientation.x,
    Pos->pose.orientation.y,
    Pos->pose.orientation.z,
    Pos->pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    emit goal(x,y,z,roll,pitch,yaw);

}

void QNode::batteryCallback(const sensor_msgs::BatteryState &message_holder)
{
    emit power(message_holder.percentage , message_holder.voltage);
}


void QNode::estimator_statusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& estimator_status)
{
/*
ESTIMATOR_ATTITUDE	True if the attitude estimate is good
ESTIMATOR_VELOCITY_HORIZ	True if the horizontal velocity estimate is good
ESTIMATOR_VELOCITY_VERT	True if the vertical velocity estimate is good
ESTIMATOR_POS_HORIZ_REL	True if the horizontal position (relative) estimate is good
ESTIMATOR_POS_HORIZ_ABS	True if the horizontal position (absolute) estimate is good
ESTIMATOR_POS_VERT_ABS	True if the vertical position (absolute) estimate is good
ESTIMATOR_POS_VERT_AGL	True if the vertical position (above ground) estimate is good
ESTIMATOR_CONST_POS_MODE	True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
ESTIMATOR_PRED_POS_HORIZ_REL	True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
ESTIMATOR_PRED_POS_HORIZ_ABS	True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
ESTIMATOR_GPS_GLITCH	True if the EKF has detected a GPS glitch
ESTIMATOR_ACCEL_ERROR	True if the EKF has detected bad accelerometer data
*/
    if( (estimator_status -> attitude_status_flag) != true )
	emit error_info("the attitude estimate is NOT good !!!");

    if( (estimator_status -> velocity_horiz_status_flag) != true )
	emit error_info("the horizontal velocity estimate is NOT good !!!");

    if( (estimator_status -> velocity_vert_status_flag) != true )
	emit error_info("the vertical velocity estimate is NOT good !!!");

    if( (estimator_status -> pos_horiz_rel_status_flag) != true )
	emit error_info("the horizontal position (relative) estimate is NOT good !!!");

    if( (estimator_status -> pos_horiz_abs_status_flag) != true )
	emit error_info("the horizontal position (absolute) estimate is NOT good !!!");

    if( (estimator_status -> pos_vert_abs_status_flag) != true )
	emit error_info("the vertical position (absolute) estimate is NOT good !!!");

    if( (estimator_status -> pos_vert_agl_status_flag) != true )
	emit error_info("the vertical position (above ground) estimate is NOT good !!!");


}
void QNode::companion_process_statusCallback(const mavros_msgs::CompanionProcessStatus::ConstPtr& companion_process_status)
{
/*
uint8 MAV_STATE_UNINIT=0
uint8 MAV_STATE_BOOT=1
uint8 MAV_STATE_CALIBRATING=2
uint8 MAV_STATE_STANDBY=3
uint8 MAV_STATE_ACTIVE=4
uint8 MAV_STATE_CRITICAL=5
uint8 MAV_STATE_EMERGENCY=6
uint8 MAV_STATE_POWEROFF=7
uint8 MAV_STATE_FLIGHT_TERMINATION=8
uint8 MAV_COMP_ID_OBSTACLE_AVOIDANCE=196
uint8 MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY=197
*/
   if( companion_process_status->state == 5)
	emit error_info("companion_process_status is CRITICAL !!!");

   if( companion_process_status->state == 6)
	emit error_info("companion_process_status is EMERGENCY !!!");

}
void QNode::statustextCallback(const mavros_msgs::StatusText::ConstPtr& statustext)
{

}

}  // namespace px4_gui
