#ifndef BASIC_WAYPOINT_PKG_PLANNER_H
#define BASIC_WAYPOINT_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class BasicPlanner {
public:
    BasicPlanner();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void setMaxSpeed(double max_v);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& goal_acc,
                        mav_trajectory_generation::Trajectory* trajectory,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

private:
    //ros::Publisher pub_markers_;
    //ros::Publisher pub_trajectory_;
    //ros::Subscriber sub_odom_;

    //ros::NodeHandle& nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H
