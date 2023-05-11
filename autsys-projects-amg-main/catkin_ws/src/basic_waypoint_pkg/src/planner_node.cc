/*
This node is for trajectory planning.
The waypoints is predefined and not dynamic.
Drone will stop at each waypoints 5 secs.
 */
#include  "ros/ros.h"
#include <planner.h>
#include <iostream>

Eigen::Vector3d goal_position, goal_velocity, goal_acceleration;
Eigen::Vector3d start_position, start_velocity;
std::vector<double> posx,posy,posz;
double x, y, z, vx, vy, vz;
int k;
double distance; 
std::string frame_id;

class traj_planning
{
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;

public:
    traj_planning()
    {
        distance = 0.2;
        frame_id = "world";
        sub = nh_.subscribe("current_state_est", 1, &traj_planning::onCurrentState, this);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
        pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
    }
    ~traj_planning(){}

    void onCurrentState(const nav_msgs::Odometry &cur_state)
    {
        x = cur_state.pose.pose.position.x;
        y = cur_state.pose.pose.position.y;
        z = cur_state.pose.pose.position.z;
        vx = cur_state.twist.twist.linear.x;
        vy = cur_state.twist.twist.linear.y;
        vz = cur_state.twist.twist.linear.z;
        //std::cout << "x: " << x << "y" << y << "z:" << z << std::endl;
        if ((abs(posx[k]-x) < 0.2) && (abs(posy[k]-y) < 0.2) && (abs(posz[k]-z) < 0.5))
        {
            BasicPlanner planner; // instantiate basic planner
            ros::Duration(5.0).sleep();
            goal_position << posx[k+1], posy[k+1], posz[k+1];  
            goal_velocity << 0.0, 0.0, 0.0;
            goal_acceleration << 0.0, 0.0, 0.0;
            start_position << x, y, z;
            start_velocity << vx, vy, vz;         
            mav_trajectory_generation::Trajectory trajectory;
            planner.planTrajectory(goal_position, goal_velocity, goal_acceleration, &trajectory, start_position, start_velocity);

            visualization_msgs::MarkerArray markers;

            mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
            pub_markers_.publish(markers);

            // send trajectory to be executed on UAV
            mav_planning_msgs::PolynomialTrajectory4D msg;
            mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
            msg.header.frame_id = "world";
            pub_trajectory_.publish(msg);

            //planner.publishTrajectory(trajectory);
            ROS_WARN_STREAM("go to next way point.");
            k = k + 1;
            if(k == 10)
            {
                ros::shutdown();
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");
    traj_planning tp;
    k = 0; 
    posx.push_back(0.15);
    posy.push_back(0);
    posz.push_back(1.15);

    posx.push_back(-17);
    posy.push_back(-5);
    posz.push_back(12);
    

    posx.push_back(-17);
    posy.push_back(15);
    posz.push_back(12);

    
    posx.push_back(-32);
    posy.push_back(15);
    posz.push_back(12);

    posx.push_back(-48);
    posy.push_back(15);
    posz.push_back(12);

     posx.push_back(-53);
    posy.push_back(15);
    posz.push_back(12);

     posx.push_back(-63);
    posy.push_back(15);
    posz.push_back(12);

     posx.push_back(-63);
    posy.push_back(-5);
    posz.push_back(12);

     posx.push_back(-40);
    posy.push_back(-5);
    posz.push_back(12);

    posx.push_back(-27);
    posy.push_back(-5);
    posz.push_back(12);

    posx.push_back(-20);
    posy.push_back(-10);
    posz.push_back(12);

    ros::spin();
    return 0;
}
