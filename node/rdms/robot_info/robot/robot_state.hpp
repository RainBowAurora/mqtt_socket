/**
 * @file robot_state.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __ROBOT_STATE_HPP__
#define __ROBOT_STATE_HPP__

#include <sensor_msgs/NavSatFix.h> //for gps info
#include <zr_msgs/battery_info.h> //for battery info
#include <std_msgs/String.h> //for light info
#include <std_msgs/Int32.h> //for delivering mode info
#include <geometry_msgs/TwistStamped.h>//for current velocity
#include <geometry_msgs/Twist.h> //for cmd vel
#include <std_msgs/Bool.h> //for emergency switch
#include <zr_msgs/locker_info.h> //for locker info
#include <nav_msgs/Odometry.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct LightData{
    std::string lightType;
    std::string state;
}; //end struct LightData

DEFINE_STRUCT_SCHEMA(LightData,\
                    DEFINE_STRUCT_FIELD(lightType, "lightType"), \
                    DEFINE_STRUCT_FIELD(state, "state"));

struct LockerData{
    int number;
    bool opened;
}; //end struct LockerData

DEFINE_STRUCT_SCHEMA(LockerData,\
                    DEFINE_STRUCT_FIELD(number, "number"), \
                    DEFINE_STRUCT_FIELD(opened, "opened"));

struct RobotStateData{
    bool alive;
    std::string latitude;
    std::string longitude;
    std::string battery;
    std::vector<LightData> lights;
    std::string drivingMode;
    std::string drivingSpeed;
    std::string spinSpeed;
    std::string estop;
    std::string serviceMode;
    std::vector<LockerData> doors;
}; //end struct RobotStateData

DEFINE_STRUCT_SCHEMA(RobotStateData, \
                    DEFINE_STRUCT_FIELD(alive, "alive"),\
                    DEFINE_STRUCT_FIELD(latitude, "latitude"),\
                    DEFINE_STRUCT_FIELD(longitude, "longitude"),\
                    DEFINE_STRUCT_FIELD(battery, "battery"),\
                    DEFINE_STRUCT_FIELD(lights, "lights"),\
                    DEFINE_STRUCT_FIELD(drivingMode, "drivingMode"),\
                    DEFINE_STRUCT_FIELD(drivingSpeed, "drivingSpeed"),\
                    DEFINE_STRUCT_FIELD(spinSpeed, "spinSpeed"),\
                    DEFINE_STRUCT_FIELD(estop, "estop"),\
                    DEFINE_STRUCT_FIELD(serviceMode, "serviceMode"),\
                    DEFINE_STRUCT_FIELD(doors, "doors"));
                    
DEFINE_STRUCT_INFO_FIELD(RobotStateData);


namespace zros{

class RobotState final: public RdmsTopicBase{
public:
    RobotState(): ros_nh_(), ros_param_("~"){

    }

    void init() override{
        RdmsTopicBase::topicInit("robot", "state");
 
        ros_param_.param<double>("/info_rate", info_rate_, 3.0);
        timer = ros_nh_.createTimer(ros::Duration(info_rate_), &RobotState::uploadMessage, this);
        sub_gps_info_ = ros_nh_.subscribe("/fix_new", 1, &RobotState::rosGpsFixCallback, this);
        sub_battery_info_ = ros_nh_.subscribe("/battery_info", 1, &RobotState::rosBatteryInfoCallback, this);
        sub_driving_mode_ = ros_nh_.subscribe("/start_flag", 1, &RobotState::rosDrivingModeCallback, this);
        // sub_driving_speed_ = ros_nh_.subscribe("/current_velocity", 1, &RobotState::rosCurrentVelocityCallback, this);
        sub_emergency_info_ = ros_nh_.subscribe("/emergency_switch", 1, &RobotState::rosEmergencySwitchCallback, this);
        sub_service_mode_ = ros_nh_.subscribe("/service_mode", 1, &RobotState::rosServiceModeCallback, this);
        sub_locker_info_ = ros_nh_.subscribe("/locker_info", 1, &RobotState::rosLockerInfoCallback, this);
        sub_cmd_vel_ = ros_nh_.subscribe("/cmd_vel", 1, &RobotState::rosCmdVelCallback, this);
        sub_front_light_ = ros_nh_.subscribe("/light_control",1, &RobotState::rosFrontLightCallback, this);
        sub_flag_light_ = ros_nh_.subscribe("/flag_light", 1, &RobotState::rosFlagLightCallback, this);
        sub_wheel_odom_ = ros_nh_.subscribe("/wheel_odom", 1, &RobotState::rosWheelOdomCallback, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::NodeHandle ros_param_;
    ros::Subscriber sub_gps_info_;
    ros::Subscriber sub_battery_info_;
    ros::Subscriber sub_light_info_;
    ros::Subscriber sub_driving_mode_;
    // ros::Subscriber sub_driving_speed_;
    ros::Subscriber sub_emergency_info_;
    ros::Subscriber sub_service_mode_;
    ros::Subscriber sub_locker_info_;
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_front_light_;
    ros::Subscriber sub_flag_light_;
    ros::Subscriber sub_wheel_odom_;
    ros::Timer timer;

    double info_rate_;
    //variable
    sensor_msgs::NavSatFix gps_info_;
    zr_msgs::battery_info battery_info_;
    std_msgs::String light_info_;
    std_msgs::Int32 delivering_mode_;
    // geometry_msgs::TwistStamped current_velocity_;
    std_msgs::Bool emergency_switch_;
    std_msgs::String service_mode_;
    zr_msgs::locker_info locker_info_;
    geometry_msgs::Twist cmd_vel_;
    std_msgs::Int32 front_light_;
    std_msgs::Int32 flag_light_;
    nav_msgs::Odometry wheel_odom_;

    void rosWheelOdomCallback(const nav_msgs::Odometry& msg){
        wheel_odom_ = msg;
    }

    void rosCmdVelCallback(const geometry_msgs::Twist& msg){
        cmd_vel_ = msg;
    }

    void rosLockerInfoCallback(const zr_msgs::locker_info& msg){
        locker_info_ = msg;
    }

    void rosServiceModeCallback(const std_msgs::String& msg){
        service_mode_ = msg;
    }

    void rosEmergencySwitchCallback(const std_msgs::Bool msg){
        emergency_switch_ = msg;
    }

    // void rosCurrentVelocityCallback(const geometry_msgs::TwistStamped& msg){
    //     current_velocity_ = msg;
    // }

    void rosDrivingModeCallback(const std_msgs::Int32 msg){
        delivering_mode_ = msg;
    }

    void rosFrontLightCallback(const std_msgs::Int32& msg){
        front_light_ = msg;
    }

    void rosFlagLightCallback(const std_msgs::Int32& msg){
        flag_light_ = msg;
    }

    void rosBatteryInfoCallback(const zr_msgs::battery_info& msg){
        battery_info_ = msg;
    }

    void rosGpsFixCallback(const sensor_msgs::NavSatFix& msg){
        gps_info_ = msg; 
    }

    std::string doubleToString(const double value, unsigned int precision){
        std::stringstream ss;
        ss << std::setiosflags(std::ios::fixed) << std::setprecision(precision)  << value;
        return ss.str();
    }

    void uploadMessage(const ros::TimerEvent& e){
        MqttInfoPayload<RobotStateData> robot_state_playload;
        robot_state_playload.clientId = getRobotID();
        robot_state_playload.timestamp = getUnixTimeStamp();
        robot_state_playload.data.alive = true;
        robot_state_playload.data.latitude = (gps_info_.status.status != -1? std::to_string(gps_info_.latitude) : "0.0");
        robot_state_playload.data.longitude = (gps_info_.status.status != -1? std::to_string(gps_info_.longitude): "0.0");
        robot_state_playload.data.battery = doubleToString(battery_info_.pct, 3);
    
        robot_state_playload.data.lights.push_back({"FRONT", (front_light_.data !=0 ? "ON": "OFF")});
        robot_state_playload.data.lights.push_back({"FLAG", (flag_light_.data !=0 ? "ON": "OFF")});

        robot_state_playload.data.drivingMode = (delivering_mode_.data == 0? "MANUAL": "AUTO");
        robot_state_playload.data.drivingSpeed = doubleToString(wheel_odom_.twist.twist.linear.x, 3);
        robot_state_playload.data.spinSpeed = doubleToString(wheel_odom_.twist.twist.angular.z, 3);
        robot_state_playload.data.estop = "NONE";
        if(emergency_switch_.data){
            robot_state_playload.data.estop = "HARDWARE";
        }
        if(delivering_mode_.data == 0 && cmd_vel_.linear.x == 0 && cmd_vel_.angular.z == 0){
            robot_state_playload.data.estop = "SOFTWARE";
            if(emergency_switch_.data){
                robot_state_playload.data.estop = "BOTH";
            }
        }

        robot_state_playload.data.serviceMode = service_mode_.data;
        robot_state_playload.data.serviceMode = "SHOP";

        for(int i = 0; i < locker_info_.locker_status.size(); i++){
            LockerData temp_locker;
            if(i > 1) break; //only use two locker
            temp_locker.number = i+1; 
            temp_locker.opened = (locker_info_.locker_status[i] == "opened"? true: false);
            robot_state_playload.data.doors.push_back(std::move(temp_locker));
        }
        publishTopic(getInfoTopicName(), robot_state_playload);
    }

}; //end class RobotState

REGIST_MQTT_TOPIC(RobotState, RobotState);

} //end namespace zros

#endif /*__ROBOT_STATE_HPP__*/