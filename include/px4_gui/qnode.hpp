#ifndef px4_gui_QNODE_HPP_
#define px4_gui_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include "rosbag/bag.h"
#include <std_msgs/UInt16.h>
#include <aerial_autonomy/VelocityYaw.h>

#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>              
#include <sensor_msgs/image_encodings.h>  
#include <visualization_msgs/Marker.h>
#include "mavros_msgs/State.h"
#include <sensor_msgs/Imu.h>
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace px4_gui {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    //bool init(const std::string &master_url, const std::string &host_url);
    bool init();
    void disinit();
    void run();
    void stop(); 
    void gripper_publish(int value);
    QMap<QString,QString> get_topic_list();
	

Q_SIGNALS:


    void Master_shutdown();
    void rosShutdown();
    void power(float p,float v);
    void state(bool arm, std::string mode);

    void position_setpoint(double x,double y,double z,double roll,double pitch ,double yaw);
    void position(double x,double y,double z,double roll,double pitch ,double yaw);

    void goal(double x,double y,double z,double roll,double pitch ,double yaw);
    
    void velocity_setpoint(double vx,double vy,double vz,double vroll,double vpitch ,double vyaw);
    void velocity(double vx,double vy,double vz,double vroll,double vpitch ,double vyaw);
    void velocity_yaw(double vx,double vy,double vz,double yaw);
    void accel(double lx,double ly,double lz);
    void gripper(int);
    
private:
    int init_argc;
    char** init_argv;
 
    ros::Subscriber pos_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber power_sub;
    ros::Subscriber state_sub;

    ros::Subscriber velocity_sub;
    ros::Subscriber accel_sub;

    ros::Subscriber position_setpoint_sub;
    ros::Subscriber velocity_setpoint_sub; 
    ros::Subscriber velocity_yaw_sub;
    ros::Subscriber gripper_sub;

    ros::Publisher pose_marker_pub;
    ros::Publisher goal_pub;
    ros::Publisher goal_marker_pub;
    ros::Publisher path_pub;
    ros::Publisher gripper_pub;

    std::string goal_pose_topic;
    std::string velocity_yaw_topic;
    std::string local_pose_topic;
    std::string state_topic;
    std::string pose_topic;
    std::string velocity_topic;
    
    std::string battery_topic;
    std::string accel_topic;

    std::string gripper_angle_topic;
    std::string gripper_topic;
    std::string gripper_command_topic;

    std::string position_setpoint_topic;
    std::string velocity_setpoint_topic;
    std::string ground_distance_topic;

    nav_msgs::Path path,goal_path;

    void gripperCallback(const std_msgs::UInt16 Command);

    void position_setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& pos);

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pos);

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pos);

    void stateCallback(const mavros_msgs::State::ConstPtr& State);
   
    void batteryCallback(const sensor_msgs::BatteryState& message_holder);

    void velocity_setpointCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity);

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity);

    void accelCallback(const sensor_msgs::Imu::ConstPtr& accel);
    
    void velocity_yawCallback(const aerial_autonomy::VelocityYaw::ConstPtr& Velocity_yaw);

    
};

}  // namespace px4_gui

#endif /* px4_gui_QNODE_HPP_ */
