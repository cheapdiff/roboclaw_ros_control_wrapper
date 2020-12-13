
#include <cmath>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "roboclaw/RoboclawMotorVelocity.h"
#include "roboclaw/RoboclawEncoderSteps.h"

class RoboClawDriver : public hardware_interface::RobotHW
{
public:
    RoboClawDriver();
    void read();
    void write();
    ros::Time getTime();
    ros::Duration getPeriod();

private:
    void odom_cb(const roboclaw::RoboclawEncoderSteps &msg);

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];

    double gear_ratio_;
    int enc_qppr_;
    double rad_per_pulse_;
    double last_pos_[2];
    ros::Time last_;

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber enc_sub_;
};

RoboClawDriver::RoboClawDriver() : cmd_{}, pos_{}, vel_{}, eff_{}, last_pos_{}
{
    hardware_interface::JointStateHandle right_state_handle("right_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
    joint_state_interface_.registerHandle(right_state_handle);

    hardware_interface::JointStateHandle left_state_handle("left_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
    joint_state_interface_.registerHandle(left_state_handle);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle right_velocity_handle(joint_state_interface_.getHandle("right_wheel_joint"), &cmd_[0]);
    velocity_joint_interface_.registerHandle(right_velocity_handle);

    hardware_interface::JointHandle left_velocity_handle(joint_state_interface_.getHandle("left_wheel_joint"), &cmd_[1]);
    velocity_joint_interface_.registerHandle(left_velocity_handle);

    registerInterface(&velocity_joint_interface_);

    cmd_pub_ = nh_.advertise<roboclaw::RoboclawMotorVelocity>("motor_cmd_vel", 10);
    enc_sub_ = nh_.subscribe("motor_enc", 10, &RoboClawDriver::odom_cb, this);

    nh_.param<double>("gear_ratio", gear_ratio_, 34.02);
    nh_.param<int>("encoder_qppr", enc_qppr_, 44);

    rad_per_pulse_ = (2 * M_PI) / (gear_ratio_ * enc_qppr_);

    last_ = ros::Time::now();
}

void RoboClawDriver::read()
{
    int right_qpps = (cmd_[0] / (2 * M_PI)) * gear_ratio_ * enc_qppr_;
    int left_qpps = (cmd_[1] / (2 * M_PI)) * gear_ratio_ * enc_qppr_;
    roboclaw::RoboclawMotorVelocity msg;
    msg.index = 0;
    msg.mot1_vel_sps = right_qpps;
    msg.mot2_vel_sps = left_qpps;

    cmd_pub_.publish(msg);
}

void RoboClawDriver::odom_cb(const roboclaw::RoboclawEncoderSteps &msg)
{
    pos_[0] = rad_per_pulse_ * msg.mot1_enc_steps;
    pos_[1] = rad_per_pulse_ * msg.mot2_enc_steps;

    ros::Time now = getTime();
    vel_[0] = (last_pos_[0] - pos_[0]) * (now - last_).toSec();
    vel_[1] = (last_pos_[1] - pos_[1]) * (now - last_).toSec();
    last_pos_[0] = pos_[0];
    last_pos_[1] = pos_[1];
    last_ = now;
}

ros::Time RoboClawDriver::getTime() { return ros::Time::now(); }
ros::Duration RoboClawDriver::getPeriod() { return ros::Duration(0.01); }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboclaw");
    ros::NodeHandle nh;

    RoboClawDriver roboclaw_driver;
    controller_manager::ControllerManager controller_manager(&roboclaw_driver, nh);

    ros::Rate rate(1.0 / roboclaw_driver.getPeriod().toSec());

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        roboclaw_driver.read();

        controller_manager.update(roboclaw_driver.getTime(), roboclaw_driver.getPeriod());
        rate.sleep();
    }
    spinner.stop();

    return 0;
}