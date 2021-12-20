#ifndef __MOVE_H__
#define __MOVE_H__


#include "vm_common.h"


class VmMove
{
private:
    
   dji_osdk_ros::FlightTaskControl control_task_;
   ros::ServiceClient task_control_client_;
   PID linear_speed_pid_;
   PID th_w_pid_;
   PID z_linear_speed_pid_;

 uint8_t stop_busy_bitflag_ = 0;
  

   Eigen::Vector4d PoseGoal_;
   Eigen::Vector4d CurrentPose_;
   Eigen::Vector4d InputVel_;
   geometry_msgs::Twist input_twist_;

   
    double linear_speed_ = 0.0;

    double_t pid_dt_ = 0.01, pid_max_ = 0.7, pid_min_ = 0.0, pid_Kp_ = 0.5, pid_Kd_ = 0.1, pid_Ki_ = 0.0;
    double_t pid_z_dt_ = 0.01, pid_z_max_ = 0.7, pid_z_min_ = 0.0, pid_z_Kp_ = 1.0, pid_z_Kd_ = 0.1, pid_z_Ki_ = 0.0;
    double_t pid_th_dt_ = 0.01, pid_th_max_ = 3.0, pid_th_min_ = 0.0, pid_th_Kp_ = 1.0, pid_th_Kd_ = 0.1, pid_th_Ki_ = 0.0;

    bool RotateAlgorithm(const double_t &goal_yaw, const double_t &yaw) ;
    double_t LinearFlightAlgorithm(const double_t &&goal_yaw);


public:
    VmMove(/* args */);
    ~VmMove();
    bool TakeOff(void);
    bool Landing(void);
    bool MoveByOdom(const uint8_t &&flight_type);
    friend void ThreePointDistance(const std::vector<std::pair<double_t, double_t>> &point, double_t &distance);
    friend void TwoPointDistance(const std::vector<std::pair<double_t, double_t>> &point, double_t &distance);
    void SetGoalPose(const double_t &&goal_x,const double_t &&goal_y,const double_t &&goal_z,const double_t &&goal_yaw){
        PoseGoal_[InfoDirType::X]=goal_x;
        PoseGoal_[InfoDirType::Y]=goal_y;
        PoseGoal_[InfoDirType::Z]=goal_z;
        PoseGoal_[InfoDirType::YAW]=goal_yaw;
    }
    bool ReturnToHome(double_t &&latitude);

    bool LandingByMarker(void);
    // 1bit : stop bit( 1 : true(stop) , 0 : false(non-stop))
    // 2bit : busy bit( 1 : true(busy) , 0 : false(not busy))
    void SetStopBusyFlag(volatile const uint8_t &bit_flag){
        stop_busy_bitflag_|=bit_flag;
    }
    uint8_t GetStopBusyFlag(void){
        return stop_busy_bitflag_;
    }
    bool MoveWaypoint(const std::shared_ptr<std::vector<Waypoint>> &&waypoints);
    bool MovebyPoseAPI(void);
    std::shared_ptr<std::vector<NorthEastCoordinate>> VmMove::GPSToOdom(const double &latitude, const double &longitude, const double &altitude );

};
VmMove::VmMove(/* args */)
{
    PoseGoal_ = Eigen::Vector4d::Zero();
    CurrentPose_ =  Eigen::Vector4d::Zero();
}

VmMove::~VmMove()
{
    ROS_INFO("VmMove class finish");
}

/**
 * @brief Takeoff function
 * 
 */

bool VmMove::ReturnToHome(double_t &&latitude){
 
    std::shared_ptr<std::vector<Waypoint>> ptr_waypoint_vector = std::make_shared<std::vector<Waypoint>>();
    ptr_waypoint_vector->clear();
    ptr_waypoint_vector->reserve(3);
    Waypoint temp_waypoint;
    //1st
    temp_waypoint.x_goal = CurrentPose_[InfoDirType::X];
    temp_waypoint.y_goal = CurrentPose_[InfoDirType::Y];
    temp_waypoint.z_goal = latitude;
    temp_waypoint.yaw_goal = CurrentPose_[InfoDirType::YAW];
    ptr_waypoint_vector->push_back(temp_waypoint);
    //2nd
    temp_waypoint.x_goal = 0;
    temp_waypoint.y_goal = 0;
    temp_waypoint.z_goal = latitude;
    temp_waypoint.yaw_goal = 0;
    ptr_waypoint_vector->push_back(temp_waypoint);
    //3rd
    temp_waypoint.x_goal = 0;
    temp_waypoint.y_goal = 0;
    temp_waypoint.z_goal = latitude_for_detection;
    temp_waypoint.yaw_goal =0;
    ptr_waypoint_vector->push_back(temp_waypoint);

    if(!MoveWaypoint(std::move(ptr_waypoint_vector))){
        ROS_ERROR_STREAM("RTH error!");
    }
    LadningByMarker();
   
}
bool VmMove::LandingByMarker(void){
    
}

bool VmMove::TakeOff(void) {
    control_task_.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_TAKEOFF;
    ROS_INFO_STREAM("Takeoff request sending ...");
    task_control_client_.call(control_task_);
    if (control_task_.response.result == true) {
        ROS_INFO_STREAM("Takeoff task successful");
        return true;
    }
    ROS_INFO_STREAM("takeoff task fail");
    return false;
}

// /**
//  * @brief position control function
//  */

bool VmMove::MovebyPoseAPI(void) {
    ROS_INFO_STREAM("Move by position offset request sending ...");
    control_task_.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    control_task_.request.joystickCommand.y = PoseGoal_[InfoDirType::X];
    control_task_.request.joystickCommand.x = PoseGoal_[InfoDirType::Y];
    control_task_.request.joystickCommand.z = PoseGoal_[InfoDirType::Z];
    control_task_.request.joystickCommand.yaw = PoseGoal_[InfoDirType::YAW];
    control_task_.request.posThresholdInM = 0.8;
    control_task_.request.yawThresholdInDeg = 1.0;
    task_control_client_.call(control_task_);
    return control_task_.response.result;
}

/**
 * @brief landing function
 */
bool VmMove::Landing(void) {
    ROS_INFO_STREAM("Land request sending ...");
    control_task_.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_LAND;
    task_control_client_.call(control_task_);
    if (control_task_.response.result == true) {
        ROS_INFO_STREAM("Land task successful");
        return true;
    }
    ROS_INFO_STREAM("Land task failed.");
    return false;
}
/**
 * @brief 
 * 
 * @param flight_type : 1. one shot  2. waypoint 
 * @return true : inside threshold area
 * @return false : Time out or ROS Mater ERROR
 */

bool VmMove::MoveByOdom(const uint8_t &&flight_type) {
    ros::Rate Move_rate(50);
    linear_speed_pid_.PID_set(pid_dt_, pid_max_, pid_min_, pid_Kp_, pid_Kd_, pid_Ki_);
    z_linear_speed_pid_.PID_set(pid_th_dt_, pid_z_max_, pid_z_min_, pid_z_Kp_, pid_z_Kd_, pid_z_Ki_);
    th_w_pid_.PID_set(pid_th_dt_, pid_th_max_, pid_th_min_, pid_th_Kp_, pid_th_Kd_, pid_th_Ki_);
    // ros::Time time_begin = ros::Time::now();
    // current_move_check_flag = true;
    uint16_t seq = 0;
    while (ros::ok()) {
        uint8_t check_bit = (GetStopBusyFlag()&bit_mask);

        if (check_bit&stop_bit) {
            return false;
        }

        // bool Vm_Basic_Command::RotateAlgorithm(double_t & goal_yaw, double_t & yaw)

        switch (seq) {
            case BasicMoveType::ROTATE: {
                double_t diff_x = PoseGoal_[InfoDirType::X] - CurrentPose_[InfoDirType::X];
                double_t diff_y = PoseGoal_[InfoDirType::Y] - CurrentPose_[InfoDirType::Y];
                double_t tmp_yaw_goal = atan2(diff_y, diff_x);
                if (RotateAlgorithm(tmp_yaw_goal, CurrentPose_[InfoDirType::YAW])) {
                    seq++;
                }
                input_twist_.linear.x = 0;
                input_twist_.linear.y = 0;
                input_twist_.linear.z = 0;
                input_twist_.angular.z =InputVel_[InfoDirType::YAW];
                break;
            }

            case BasicMoveType::LINEAR_MOVE: {
                double_t diff_x = PoseGoal_[InfoDirType::X] -  CurrentPose_[InfoDirType::X];
                double_t diff_y = PoseGoal_[InfoDirType::Y] -  CurrentPose_[InfoDirType::Y];
                double_t tmp_yaw_goal = atan2(diff_y, diff_x);
                double_t distance = LinearFlightAlgorithm(std::move(tmp_yaw_goal));
                if (distance < THREE_POINT_DISTANCE) {
                    if (flight_type == LinearFlightType::WAYPOINT){
                        return true;
                    } else {
                        seq++;
                    }
                }
                input_twist_.linear.x = InputVel_[InfoDirType::X];
                input_twist_.linear.y = InputVel_[InfoDirType::Y];
                input_twist_.linear.z = InputVel_[InfoDirType::Z];
                input_twist_.angular.z = InputVel_[InfoDirType::YAW];
               
                break;
            }
            case BasicMoveType::FINAL_ROTATE: {
                if (RotateAlgorithm(PoseGoal_[InfoDirType::YAW], CurrentPose_[InfoDirType::YAW])) {
                    // current_move_check_flag = false;
                    // satisfy_th = false;
                    // Arrive_Flag = false;

                    return true;
                }
                input_twist_.linear.x = 0;
                input_twist_.linear.y = 0;
                input_twist_.linear.z = 0;
                input_twist_.angular.z = InputVel_[InfoDirType::YAW];;
                break;
            }
        }
    }
}



std::shared_ptr<std::vector<NorthEastCoordinate>> VmMove::GPSToOdom(const double &latitude, const double &longitude, const double &altitude ){
   double northing, easting;
   std::string zone;
   gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
   
    std::shared_ptr<std::vector<NorthEastCoordinate>> NEC_vector = std::make_shared<std::vector<NorthEastCoordinate>>();

   x_goal = NEC_vector->easting;
   y_goal = NEC_vector->northing;
   z_goal = NEC_vector->altitude;

    return NEC_vector;
 }




bool VmMove::RotateAlgorithm(const double_t &goal_yaw,const  double_t &yaw) {
    double_t fabs_yaw = fabs(goal_yaw - yaw);
    double_t diff_yaw = goal_yaw - yaw;
    // turn_speed_ = 

    if (fabs_yaw > M_PI) {
        if (RAD_360 - fabs_yaw < SAT_RANGE_YAW) {
            InputVel_[InfoDirType::YAW] = 0;
            
            return true;
        } else {
            if (diff_yaw > 0) {             
                InputVel_[InfoDirType::YAW] =th_w_pid_.calculate(0, fabs_yaw);
            } else {
                InputVel_[InfoDirType::YAW] = (-1 * th_w_pid_.calculate(0, fabs_yaw));                
            }
           
        }
    } else {
        if (fabs_yaw < SAT_RANGE_YAW) {
            InputVel_[InfoDirType::YAW] = 0;
            
            return true;
        } else {
            if (diff_yaw > 0) {               
                InputVel_[InfoDirType::YAW] = (-1 * th_w_pid_.calculate(0, fabs_yaw));
            } else {              
                InputVel_[InfoDirType::YAW] = th_w_pid_.calculate(0, fabs_yaw);
            }
           
        }
    }
}

double_t VmMove::LinearFlightAlgorithm(const double_t &&goal_yaw) {
    Eigen::MatrixXf Tr(2, 2), pre_pose(2, 1), Tr_pose(2, 1), pre_goal_pose(2, 1), Tr_goal_pose(2, 1);
    double_t th = - CurrentPose_[InfoDirType::YAW];
    Tr << cos(th), -sin(th),
        sin(th), cos(th);
    pre_pose <<  CurrentPose_[InfoDirType::X],  CurrentPose_[InfoDirType::Y];
    Tr_pose = Tr * pre_pose;

    double_t transfer_pose_x = Tr_pose(0);
    double_t transfer_pose_y = Tr_pose(1);

    pre_goal_pose << PoseGoal_[InfoDirType::X], PoseGoal_[InfoDirType::Y];
    Tr_goal_pose = Tr * pre_goal_pose;

    double_t transfer_pose_x_goal = Tr_goal_pose(0);
    double_t transfer_pose_y_goal = Tr_goal_pose(1);

    std::vector<std::pair<double_t, double_t>> point;

    point.emplace_back(std::make_pair(transfer_pose_x, transfer_pose_x_goal));
    point.emplace_back(std::make_pair(transfer_pose_y, transfer_pose_y_goal));
   
    point.emplace_back(std::make_pair( CurrentPose_[InfoDirType::Z], PoseGoal_[InfoDirType::Z]));

    double_t distance;
    ThreePointDistance(point, distance);
    double_t x_y_distance;
    TwoPointDistance(point, x_y_distance);
   
    double_t diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
    double_t diff_pose_y = transfer_pose_y_goal - transfer_pose_y;

   

    double_t diff_pose_z =  PoseGoal_[InfoDirType::Z] -  CurrentPose_[InfoDirType::Z];

    double_t tan_degree = atan2(diff_pose_y, diff_pose_x);

    double_t &&speed_pid = linear_speed_pid_.calculate(0.0, x_y_distance);
    double_t &&z_speed_pid = z_linear_speed_pid_.calculate(0.0, fabs(diff_pose_z));
    if (diff_pose_z > 0) {
         InputVel_[InfoDirType::Z] = z_speed_pid;
    } else if (diff_pose_z < 0) {
         InputVel_[InfoDirType::Z] = (-1) * z_speed_pid;
    }
   
    InputVel_[InfoDirType::Y] = (speed_pid)*sin(tan_degree);
    InputVel_[InfoDirType::X] = (speed_pid)*cos(tan_degree);
   
    RotateAlgorithm( PoseGoal_[InfoDirType::YAW], CurrentPose_[InfoDirType::YAW]);

    
    return distance;
}

bool VmMove::MoveWaypoint(const std::shared_ptr<std::vector<Waypoint>> &&waypoints) {
    for (uint8_t i = 0; i < waypoints->size(); i++) {
        PoseGoal_[InfoDirType::X] = waypoints->at(i).x_goal;
        PoseGoal_[InfoDirType::Y] = waypoints->at(i).y_goal;
        PoseGoal_[InfoDirType::Z] = waypoints->at(i).z_goal;
   
        if (i == waypoints->size() - 1) {
            if (!MoveByOdom(LinearFlightType::FINAL_POSE)) {
                return false;
            } else {
                return true;
            }
        }
        if (!MoveByOdom(LinearFlightType::WAYPOINT)) {
            return false;
        }
    }
}



#endif
