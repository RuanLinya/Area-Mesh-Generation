#include <planner.h>

BasicPlanner::BasicPlanner(){
    max_v_ = 5;
    max_a_ = 2;
} 


void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}


bool BasicPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    const Eigen::VectorXd& goal_acc,
                                    mav_trajectory_generation::Trajectory* trajectory,
                                    const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension),end(dimension); //middle(dimension),


    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(start_pos,
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel);

    // add waypoint to list
    vertices.push_back(start);

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, (start_pos + goal_pos)/2);
    vertices.push_back(middle);

    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to zero
    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      goal_acc);

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

// bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
//     // send trajectory as markers to display them in RVIZ
//     visualization_msgs::MarkerArray markers;
//     double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
//     std::string frame_id = "world";

//     mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
//     pub_markers_.publish(markers);

//     // send trajectory to be executed on UAV
//     mav_planning_msgs::PolynomialTrajectory4D msg;
//     mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
//     msg.header.frame_id = "world";
//     pub_trajectory_.publish(msg);

//     return true;
// }
