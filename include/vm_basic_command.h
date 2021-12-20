/**
 * 
 * @author Ethan.lim
 * @brief this package name is vm basic flight
 *        realizing move by pose commanded by user
 *        and exception process 
 *        Next step : sensor fusion IMU & GPS using EKF , Lane traking and obstacle avoid navigation 
 * @version 0.1
 * @date 2021-01-26
 *  * 
 * 
 */

#ifndef __COMMAND_H__
#define __COMMAND_H__


#include "vm_move_function.h"




const std::string error_msg = R"(
   Command is not correct!!!!
)";

class VmDjiM300 {
   private:
    /* data */
    struct Point {
        double x, y, z;
    };

    struct RTH_Point {
        double x, y, z = 5.0, yaw = 0.0;
    };
    struct Transfer_Position {
        double pre_pose_x, pre_pose_y, pre_pose_z;
        double transfer_pose_x, transfer_pose_y, transfer_pose_z;
    };

    ros::NodeHandle nh;
    ros::Subscriber gps_sub, keyboard_sub, gps_odom_sub, imu_odom_sub, ar_marker_sub, battery_state_sub, gps_health_sub, flight_status_sub, aruco_marker_pose_sub, velocity_sub, acceleration_sub, local_pose_sub;

    ros::ServiceClient  set_go_home_altitude_client, enable_upward_avoid_client,
        get_avoid_enable_client, emergency_brake_client, obtain_ctrl_authority_client, take_picture_oneshot_pub;

    ros::Subscriber lidar_sub, lidar_sub_3D;

   
    dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
    dji_osdk_ros::SetAvoidEnable upward_avoid_req;
    dji_osdk_ros::GetAvoidEnable getAvoidEnable;

    ros::Publisher cmd_vel_pub;
    
   
    std_srvs::SetBool takepicture_bool;
    sensor_msgs::LaserScan lidar;
   
    std::string key;
    std::string command_line(void);
    
    RTH_Point rth_point;
   
    std::shared_ptr<VmMove> ptr_move_m300_ = std::make_shared<VmMove>();
   
    bool bat_flag = false, gps_flag = false, flight_flag = false;  //, Arrive_Flag = false;
   
    bool stop_flag = false;
    bool current_move_check_flag = false;
    uint8_t emergency_flag = RTH;
    bool Marker_Landing_Working_ = false;

    
    bool CheckParamBusyException(const uint16_t& param_num, const std::vector<std::string> &param,const uint8_t &type = LinearFlightType::SIMPLE);

   
    void GetCmdCallback(const std_msgs::String::ConstPtr &msg);
    
   protected:
    double x_vel = 0, y_vel = 0, z_vel = 0, th_vel = 0;
    double x_vel_measure = 0, y_vel_measure = 0, z_vel_measure = 0;
    double marker_x = 0, marker_y = 0, marker_z = 0, marker_th = 0, marker_th_deg = 0;
    double aruco_x = 0, aruco_y = 0, aruco_z = 0;

    double x = 0, y = 0, z = 0, th_rad = 0, th_deg = 0, roll_rad = 0, pitch_rad = 0;

   

    double latitude, longitude, altitude;

    double speed = 0.0;
    double turn = 0.0;
    double marker_speed = 0.0;
    double marker_turn = 0.0;

    double z_vel_rate = 0.005, speed_rate = 0.005, turn_rate = 0.01;
    uint8_t marker_detect_cnt = 0, Imu_acc_cnt = 0, max_count = 0;
    int pic_count = 0;
    double acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
    bool marker_detect_flag = true;
    bool Lidar_Emergency_Flag = false;
    ros::Time aruco_time_begin;
    uint8_t X_n, X_m;

    std::vector<double> Imu_roll_vec, Imu_pitch_vec, Imu_yaw_vec;  //,Imu_x_acc,Imu_y_acc,Imu_z_acc;
    std::vector<double> Odom_x_vec, Odom_y_vec, Odom_z_vec;

    Eigen::MatrixXf A_EKF, Q_EKF, R_EKF, H_EKF, X_EKF, Z_EKF, P_EKF, Tr, Xp_EKF, Pp_EKF, K_EKF;
    Eigen::MatrixXf A_UKF, Q_UKF, R_UKF, H_UKF, X_UKF, Z_UKF, P_UKF, Xp_UKF, Pp_UKF, K_UKF, Pz_UKF, Zp_UKF, Xi, W;

    // Eigen::MatrixXf A(6,6);
    ros::Time filter_begin, pid_dt_begin;
    Eigen::Vector3d Lidar_Scan_Dis;
    Eigen::Vector3d Lidar_Scan_Zero;

   public:
    VmDjiM300(/* args */);
    ~VmDjiM300();

    bool Marker_Detect_flag = false;
    uint8_t Imu_Start_flag_cnt = 0, Odom_Start_flag_cnt = 0;
};

VmDjiM300::VmDjiM300(/* args */) {
   

    /***publish**/
   
    keyboard_sub = nh.subscribe("keyboard_command", 1, &VmDjiM300::GetCmdCallback, this);
    

    /**init**/
   
}

VmDjiM300::~VmDjiM300() {
    ROS_INFO("Basic_command Node close");
}

#endif
