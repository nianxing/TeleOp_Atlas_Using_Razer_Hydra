/* 
 * File:   hydra_move_group.h
 * Author: nian
 *
 * Created on May 17, 2015, 6:27 PM
 */

#ifndef HYDRA_MOVE_GROUP_H
#define	HYDRA_MOVE_GROUP_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <atlas_msgs/AtlasState.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <atlas_conversions/joint_names.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <kdl/frames.hpp>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <razer_hydra/Hydra.h>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <interaction_cursor_msgs/InteractionCursorFeedback.h>


#include <eigen3/Eigen/Geometry>

class UpperBodyPlanner: public move_group_interface::MoveGroup {
public:
    UpperBodyPlanner(const std::string &group, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
                       const ros::Duration &wait_for_server = ros::Duration(0, 0));
    UpperBodyPlanner(const Options& opt, const boost::shared_ptr<tf::Transformer>& tf = boost::shared_ptr<tf::Transformer>(), 
                       const ros::Duration& wait_for_server = ros::Duration(0, 0));
    virtual ~UpperBodyPlanner();
    bool checkConstructorStatus(const std::string &link_name);
    void updateRobotState();
    void updateEndEffectorState(const std::string &link_name);
    void jointValue_moveTo(const Eigen::VectorXd &joint_value);
    void addPose_moveTo(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& end_pose, const int &step_num, std::vector<geometry_msgs::Pose> &pose_sequence);
    void pose_moveTo(const std::string &link_name, const Eigen::Affine3d& desired_pose, const int &step_num, std::vector<geometry_msgs::Pose> &pose_sequence);
    void pose_moveTo2(const std::string &link_name, const Eigen::Affine3d& current_pose,const Eigen::Affine3d& desired_pose, const int &step_num, std::vector<geometry_msgs::Pose> &pose_sequence);
    void circlePose_moveTo(const std::string &link_name, const double &radius, const int &step_num, const Eigen::Matrix3d &R, std::vector<geometry_msgs::Pose> &pose_sequence);
    void circlePose_moveTo2(const std::string &link_name, const double &radius, const int &step_num, const Eigen::Matrix3d &R, std::vector<geometry_msgs::Pose> &pose_sequence);
    bool previewPathPlan(const std::vector<geometry_msgs::Pose> &pose_sequence, move_group_interface::MoveGroup::Plan &plan);
    bool setMovementVelocity(const double &velocity);
    bool setForceTorqueReference(const std::string &name, Eigen::Vector3d &force, Eigen::Vector3d &torque);
    Eigen::Vector3d getForceDifference(const std::string &name, const Eigen::Vector3d &force_reference);
    Eigen::Vector3d getTorqueDifference(const std::string &name, const Eigen::Vector3d &torque_reference);
    static Eigen::Matrix3d RPY2EigenMat(const Eigen::Vector3d &angle);
    static Eigen::Matrix3d tfMat2Eigen3d(const tf::Matrix3x3 &inMat_);
    static Eigen::Vector3d tfVec2Eigen3d(const tf::Vector3 &inVec_);
    static Eigen::VectorXd stdVec2EigenXd(const std::vector<double>& inVec_);
    static Eigen::Quaterniond Rmat2Quaternion(const Eigen::Matrix3d &inMat_);
    static std::vector<double> EigenXd2stdVec(const Eigen::VectorXd &inVec_);
    static geometry_msgs::Pose Eigen2msgPose(const Eigen::Vector3d &translation, const Eigen::Quaterniond &q);
    static void msgPose2Eigen(const geometry_msgs::Pose &input, Eigen::Vector3d &translation, Eigen::Quaterniond &q);
    static void msgPose2Eigen(const geometry_msgs::Pose &input, Eigen::Vector3d &translation, Eigen::Matrix3d &rotation);
    void printOutJoints();
    void printOutVariables();
    Eigen::Affine3d getPose(const std::string &link_name);
    
private:
    void getAtlasStateCB(const atlas_msgs::AtlasStateConstPtr &js);
    bool checkTrajectory(const moveit_msgs::RobotTrajectory &input_trajectory, moveit_msgs::RobotTrajectory &output_trajectory);
    void printOutQuatern(const Eigen::Quaterniond &q);
    void forceFilter(const Eigen::Vector3d &input, Eigen::Vector3d &output, Eigen::Vector3d &buffer, const double &factor);
    
    ros::NodeHandle n;
    ros::Subscriber subState;
    ros::Publisher PubDisplay;
    Eigen::VectorXd jointAngles;
    Eigen::Vector3d l_hand_force_raw;
    Eigen::Vector3d l_hand_torque_raw;
    Eigen::Vector3d r_hand_force_raw;
    Eigen::Vector3d r_hand_torque_raw;
    Eigen::Vector3d l_hand_force;
    Eigen::Vector3d l_hand_torque;
    Eigen::Vector3d r_hand_force;
    Eigen::Vector3d r_hand_torque;
    Eigen::Affine3d end_effector_state;
    moveit::core::RobotStatePtr kinematic_state;
    tf::StampedTransform eef_wrt_pelvis_transform;
    tf::TransformListener tf_listener;
    double path_step;
    double move_velocity;
    double filter_threshold;
    bool singularity_check;
    Eigen::Vector3d l_hand_force_buffer;
    Eigen::Vector3d l_hand_torque_buffer;
    Eigen::Vector3d r_hand_force_buffer;
    Eigen::Vector3d r_hand_torque_buffer;
};


#endif	/* HYDRA_MOVE_GROUP_H */

