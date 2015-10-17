/* 
 * File:   upp_body_planner.cpp
 * Author: checco
 *
 * Created on March 12, 2015, 12:52 PM
 */

#include "hydra_move_group.h"


UpperBodyPlanner::UpperBodyPlanner(const std::string &group, const boost::shared_ptr<tf::Transformer> &tf, const ros::Duration &wait_for_server) : 
move_group_interface::MoveGroup::MoveGroup(group, tf, wait_for_server) {
    jointAngles.resize(NJoints);
    jointAngles[0] = -100;
    subState = n.subscribe("/atlas/atlas_state", 1, &UpperBodyPlanner::getAtlasStateCB, this);
    PubDisplay = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    kinematic_state = this->getCurrentState();
    move_velocity = 0.05;
    filter_threshold = 5;
    path_step = 0.005;
    l_hand_force_buffer = Eigen::VectorXd::Zero(3);
    l_hand_torque_buffer = Eigen::VectorXd::Zero(3);
    r_hand_force_buffer = Eigen::VectorXd::Zero(3);
    r_hand_torque_buffer = Eigen::VectorXd::Zero(3);
}

UpperBodyPlanner::UpperBodyPlanner(const Options& opt, const boost::shared_ptr<tf::Transformer>& tf, const ros::Duration& wait_for_server) :
move_group_interface::MoveGroup::MoveGroup(opt, tf, wait_for_server) {
    jointAngles.resize(NJoints);
    jointAngles[0] = -100;
    subState = n.subscribe("/atlas/atlas_state", 1, &UpperBodyPlanner::getAtlasStateCB, this);
    PubDisplay = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    kinematic_state = this->getCurrentState();
    move_velocity = 0.05;
    filter_threshold = 5;
    path_step = 0.005;
    l_hand_force_buffer = Eigen::VectorXd::Zero(3);
    l_hand_torque_buffer = Eigen::VectorXd::Zero(3);
    r_hand_force_buffer = Eigen::VectorXd::Zero(3);
    r_hand_torque_buffer = Eigen::VectorXd::Zero(3);
}

UpperBodyPlanner::~UpperBodyPlanner() {}

bool UpperBodyPlanner::checkConstructorStatus(const std::string &link_name) {
    bool tf_listener_ready = false;
    bool atlas_state_ready = true;
    try {
        tf_listener_ready = true;
        tf_listener.lookupTransform("/pelvis", link_name, ros::Time(0), eef_wrt_pelvis_transform);
    }
    catch (tf::TransformException ex) {
        tf_listener_ready = false;
    }
    if (jointAngles[0] == -100) {
        ROS_INFO("Waiting for updating atlas state.");
        atlas_state_ready = false;
    }
    if (tf_listener_ready && atlas_state_ready) return true;
    else return false;
}

//mapping atlas/atlas_state to move_group
void UpperBodyPlanner::updateRobotState() {
    //pelvis and torso
    for (int i = 0; i < 3; i++) {
        kinematic_state->setVariablePosition(i + 3, jointAngles(i));
    }
    //neck
    kinematic_state->setVariablePosition(52, jointAngles(3));
    //left arm
    for (int i = 0; i < 7; i++) {
        kinematic_state->setVariablePosition(i + 6, jointAngles(i + 16));
    }
    //right arm
    for (int i = 0; i < 7; i++) {
        kinematic_state->setVariablePosition(i + 54, jointAngles(i + 23));
    }
    //left leg
    for (int i = 0; i < 6; i++) {
        kinematic_state->setVariablePosition(i + 61, jointAngles(i + 4));
    }
    //right leg
    for (int i = 0; i < 6; i++) {
        kinematic_state->setVariablePosition(i + 67, jointAngles(i + 10));
    }
    kinematic_state->update();
    singularity_check = true;
}

void UpperBodyPlanner::updateEndEffectorState(const std::string &link_name) {
    tf_listener.lookupTransform("/pelvis", link_name, ros::Time(0), eef_wrt_pelvis_transform);
    end_effector_state.translation() = tfVec2Eigen3d(eef_wrt_pelvis_transform.getOrigin());
    end_effector_state.linear() = tfMat2Eigen3d(eef_wrt_pelvis_transform.getBasis());
    std::cout << "Current end effector state is: " << std::endl;
    std::cout << end_effector_state.translation() << std::endl;
    std::cout << end_effector_state.rotation() << std::endl;
}

void UpperBodyPlanner::jointValue_moveTo(const Eigen::VectorXd& joint_value) {
    if (joint_value.size() != 7) ROS_ERROR("Wrong size of joint value.");
    else {
        std::vector<std::string> joints = getJoints();
        if (joints.size() == 0) ROS_ERROR("Can not get joints.");
        else if (joints.size() < 7) ROS_WARN("Wrong joint size, please double check.");
        else {
            for (int i = 0; i < 7; i++) {
                setJointValueTarget(joints[i], joint_value(i));
            }
        }
    }
}

Eigen::Affine3d UpperBodyPlanner::getPose(const std::string &link_name){
    Eigen::Affine3d current_pose = kinematic_state->getGlobalLinkTransform(link_name);
    return current_pose;
}

void UpperBodyPlanner::addPose_moveTo(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& end_pose, const int& step_num, std::vector<geometry_msgs::Pose>& pose_sequence) {
    Eigen::Quaterniond start_q = Rmat2Quaternion(start_pose.rotation());
    Eigen::Quaterniond end_q = Rmat2Quaternion(end_pose.rotation());
    Eigen::Quaterniond step_q;
    
    std::cout << "**********************************" << std::endl;
    std::cout << "start_pose translation is: " << start_pose.translation().transpose() << std::endl;
    std::cout << "start_pose rotation is: " << std::endl;
    std::cout << start_pose.rotation() << std::endl;
    std::cout << "start_pose quaternion is: "<< std::endl;
    std::cout << "w = " << start_q.w() << ", x = " << start_q.x() << ", y = " << start_q.y() << ", z = " << start_q.z() << std::endl;
    std::cout << "end_pose translation is: " << end_pose.translation().transpose() << std::endl;
    std::cout << "end_pose rotation is: " << std::endl;
    std::cout << end_pose.rotation() << std::endl;
    std::cout << "Desired quaternion is: " << std::endl;
    std::cout << "w = " << end_q.w() << ", x = " << end_q.x() << ", y = " << end_q.y() << ", z = " << end_q.z() << std::endl;
    std::cout << "**********************************" << std::endl;
    
    Eigen::Vector3d step_translation_movement = (end_pose.translation() - start_pose.translation()) / step_num;
    for (int i = 0; i < step_num; i++) {
        Eigen::Affine3d step_target;
        step_target.translation() = start_pose.translation() + (i + 1) * step_translation_movement;
        step_q = start_q.slerp((i + 1.0) / step_num, end_q);
        geometry_msgs::Pose target_pose = Eigen2msgPose(step_target.translation(), step_q);
        pose_sequence.push_back(target_pose);
    }
}


void UpperBodyPlanner::pose_moveTo(const std::string &link_name, const Eigen::Affine3d& desired_pose, const int &step_num, std::vector<geometry_msgs::Pose> &pose_sequence) {
    Eigen::Affine3d current_pose = kinematic_state->getGlobalLinkTransform(link_name);
    Eigen::Quaterniond current_q = Rmat2Quaternion(current_pose.rotation());
    Eigen::Quaterniond desired_q = Rmat2Quaternion(desired_pose.rotation());
    Eigen::Quaterniond step_q;
    
    std::cout << "**********************************" << std::endl;
    std::cout << "end_effector_name is: " << link_name << std::endl;
    std::cout << "Current translation is: " << current_pose.translation().transpose() << std::endl;
    std::cout << "current rotation is: " << std::endl;
    std::cout << current_pose.rotation() << std::endl;
    std::cout << "current quaternion is: "<< std::endl;
    std::cout << "w = " << current_q.w() << ", x = " << current_q.x() << ", y = " << current_q.y() << ", z = " << current_q.z() << std::endl;
    std::cout << "Desired translation is: " << desired_pose.translation().transpose() << std::endl;
    std::cout << "Desired rotation is: " << std::endl;
    std::cout << desired_pose.rotation() << std::endl;
    std::cout << "Desired quaternion is: " << std::endl;
    std::cout << "w = " << desired_q.w() << ", x = " << desired_q.x() << ", y = " << desired_q.y() << ", z = " << desired_q.z() << std::endl;
    std::cout << "**********************************" << std::endl;
    
    Eigen::Vector3d step_translation_movement = (desired_pose.translation() - current_pose.translation()) / step_num;
    for (int i = 0; i < step_num; i++) {
        Eigen::Affine3d step_target;
        step_target.translation() = current_pose.translation() + (i + 1) * step_translation_movement;
        step_q = current_q.slerp((i + 1.0) / step_num, desired_q);
        geometry_msgs::Pose target_pose = Eigen2msgPose(step_target.translation(), step_q);
        pose_sequence.push_back(target_pose);
    }
}

void UpperBodyPlanner::pose_moveTo2(const std::string &link_name, 
                                   const Eigen::Affine3d& current_pose,
                                   const Eigen::Affine3d& desired_pose, const int &step_num, 
                                   std::vector<geometry_msgs::Pose> &pose_sequence) {
    //Eigen::Affine3d current_pose = kinematic_state->getGlobalLinkTransform(link_name);
    Eigen::Quaterniond current_q = Rmat2Quaternion(current_pose.rotation());
    Eigen::Quaterniond desired_q = Rmat2Quaternion(desired_pose.rotation());
    Eigen::Quaterniond step_q;
    
    std::cout << "**********************************" << std::endl;
    std::cout << "end_effector_name is: " << link_name << std::endl;
    std::cout << "Current translation is: " << current_pose.translation().transpose() << std::endl;
    std::cout << "current rotation is: " << std::endl;
    std::cout << current_pose.rotation() << std::endl;
    std::cout << "current quaternion is: "<< std::endl;
    std::cout << "w = " << current_q.w() << ", x = " << current_q.x() << ", y = " << current_q.y() << ", z = " << current_q.z() << std::endl;
    std::cout << "Desired translation is: " << desired_pose.translation().transpose() << std::endl;
    std::cout << "Desired rotation is: " << std::endl;
    std::cout << desired_pose.rotation() << std::endl;
    std::cout << "Desired quaternion is: " << std::endl;
    std::cout << "w = " << desired_q.w() << ", x = " << desired_q.x() << ", y = " << desired_q.y() << ", z = " << desired_q.z() << std::endl;
    std::cout << "**********************************" << std::endl;
    
    Eigen::Vector3d step_translation_movement = (desired_pose.translation() - current_pose.translation()) / step_num;
    for (int i = 0; i < step_num; i++) {
        Eigen::Affine3d step_target;
        step_target.translation() = current_pose.translation() + (i + 1) * step_translation_movement;
        step_q = current_q.slerp((i + 1.0) / step_num, desired_q);
        geometry_msgs::Pose target_pose = Eigen2msgPose(step_target.translation(), step_q);
        pose_sequence.push_back(target_pose);
    }
}

void UpperBodyPlanner::circlePose_moveTo(const std::string& link_name, const double& radius, const int& step_num, const Eigen::Matrix3d& R, std::vector<geometry_msgs::Pose>& pose_sequence) {
    Eigen::Affine3d current_pose = kinematic_state->getGlobalLinkTransform(link_name);
    Eigen::Quaterniond current_q = Rmat2Quaternion(current_pose.rotation());
    Eigen::Quaterniond step_q;
    
    std::cout << "**********************************" << std::endl;
    std::cout << "end_effector_name is: " << link_name << std::endl;
    std::cout << "Current translation is: " << std::endl;
    std::cout << current_pose.translation() << std::endl;
    std::cout << "current rotation is: " << std::endl;
    std::cout << current_pose.rotation() << std::endl;
    std::cout << "The circle's radius is: " << radius << std::endl;
    
    double theta;
    for (int i = 0; i < step_num; i++) {
        Eigen::Affine3d step_target;
        //theta = (M_PI * (i + 1)) / (step_num * 2);
        theta = 2 * M_PI * (i + 1) / step_num;
        Eigen::Vector3d movement_wrt_door = Eigen::VectorXd::Zero(3);
        //movement_wrt_door(0) = (1 - cos(theta)) * radius;
        //movement_wrt_door(2) = -sin(theta) * radius;
        movement_wrt_door(1) = (1 - cos(theta)) * radius;
        movement_wrt_door(2) = sin(theta) * radius;
                
        //std::cout << "DEBUG: theta is: " << theta << std::endl;
        //std::cout << "DEBUG: movement_wrt_door is: " << movement_wrt_door.transpose() << std::endl;  
        
        step_target.translation() = current_pose.translation() + R * movement_wrt_door;
        step_q = current_q;
        geometry_msgs::Pose target_pose = Eigen2msgPose(step_target.translation(), step_q);
        pose_sequence.push_back(target_pose);
    }
}

void UpperBodyPlanner::circlePose_moveTo2(const std::string& link_name, 
                                        const double& radius, 
                                        const int& step_num, 
                                        const Eigen::Matrix3d& R,   
                                            std::vector<geometry_msgs::Pose>& pose_sequence) {
    Eigen::Affine3d current_pose = kinematic_state->getGlobalLinkTransform(link_name);
    Eigen::Quaterniond current_q = Rmat2Quaternion(current_pose.rotation());
    Eigen::Quaterniond step_q;
    
    std::cout << "**********************************" << std::endl;
    std::cout << "end_effector_name is: " << link_name << std::endl;
    std::cout << "Current translation is: " << std::endl;
    std::cout << current_pose.translation() << std::endl;
    std::cout << "current rotation is: " << std::endl;
    std::cout << current_pose.rotation() << std::endl;
    std::cout << "The circle's radius is: " << radius << std::endl;
    
    double theta;
    for (int i = 0; i < step_num; i++) {
        if (i >= step_num / 2 ){
            Eigen::Affine3d step_target;
            //theta = (M_PI * (i + 1)) / (step_num * 2);
            theta = 2 * M_PI * (i + 1) / step_num;
            Eigen::Vector3d movement_wrt_door = Eigen::VectorXd::Zero(3);
            //movement_wrt_door(0) = (1 - cos(theta)) * radius;
            //movement_wrt_door(2) = -sin(theta) * radius;

            movement_wrt_door(1) = (1 - cos(theta)) * radius; //y
            movement_wrt_door(2) = sin(theta) * radius;          //z

            std::cout << "DEBUG: Able to perform step number - " << i << std::endl;

            //std::cout << "DEBUG: theta is: " << theta << std::endl;
            //std::cout << "DEBUG: movement_wrt_door is: " << movement_wrt_door.transpose() << std::endl;  

            step_target.translation() = current_pose.translation() + R * movement_wrt_door;
            step_q = current_q;
            geometry_msgs::Pose target_pose = Eigen2msgPose(step_target.translation(), step_q);
            pose_sequence.push_back(target_pose);
        }
    }
}

bool UpperBodyPlanner::previewPathPlan(const std::vector<geometry_msgs::Pose>& pose_sequence, move_group_interface::MoveGroup::Plan &plan) {
    std::cout << "Adding " << pose_sequence.size() << " points." << std::endl;
    moveit_msgs::RobotTrajectory trajectory;
    moveit::core::robotStateToRobotStateMsg(*kinematic_state, plan.start_state_);
    double fraction = this->computeCartesianPath(pose_sequence, path_step, 0.0, trajectory);
    std::cout << "Fraction is: " << fraction << std::endl;
    std::cout << "Current position tolerance is: " << this->getGoalPositionTolerance() << std::endl;
    std::cout << "Current orientation tolerance is: " << this->getGoalOrientationTolerance() << std::endl;
    if (fraction != 1.0) ROS_WARN("The plan is not perfect. Current fraction isn't equal to 1.0");
    checkTrajectory(trajectory, plan.trajectory_);
    if (singularity_check == false) ROS_WARN("The plan didn't pass singularity check. Suggest to replan.");
    ROS_INFO("Preview planning...");
    if (fraction != 1.0 || singularity_check == false) return false;
    return true;
}

bool UpperBodyPlanner::setMovementVelocity(const double& velocity) {
    if (velocity <= 0 || velocity >= 0.1) {
        ROS_ERROR("invalid velocity input.");
        return false;
    }
    else {
        move_velocity = velocity;
        std::cout << "Current move velocity is: " << move_velocity << std::endl;
        return true;
    }
}

bool UpperBodyPlanner::setForceTorqueReference(const std::string& name, Eigen::Vector3d& force, Eigen::Vector3d& torque) {
    if (name == "l_hand") {
        force = l_hand_force;
        torque = l_hand_torque;
        return true;
    }
    else if (name == "r_hand") {
        force = r_hand_force;
        torque = r_hand_torque;
        return true;
    }
    else {
        ROS_ERROR("Wrong input name.");
        return false;
    }
}

Eigen::Vector3d UpperBodyPlanner::getForceDifference(const std::string& name, const Eigen::Vector3d& force_reference) {
    if (name == "l_hand") return (l_hand_force - force_reference);
    else if (name == "r_hand") return (r_hand_force - force_reference);
    else return Eigen::VectorXd::Zero(3);
}

Eigen::Vector3d UpperBodyPlanner::getTorqueDifference(const std::string& name, const Eigen::Vector3d& torque_reference) {
    if (name == "l_hand") return (l_hand_torque - torque_reference);
    else if (name == "r_hand") return (r_hand_torque - torque_reference);
    else return Eigen::VectorXd::Zero(3);
}

void UpperBodyPlanner::getAtlasStateCB(const atlas_msgs::AtlasStateConstPtr& js) {
    std::vector<double> joint_position(js->position.begin(), js->position.end());
    jointAngles = stdVec2EigenXd(joint_position);
    l_hand_force_raw(0) = js->l_hand.force.x;
    l_hand_force_raw(1) = js->l_hand.force.y;
    l_hand_force_raw(2) = js->l_hand.force.z;
    l_hand_torque_raw(0) = js->l_hand.torque.x;
    l_hand_torque_raw(1) = js->l_hand.torque.y;
    l_hand_torque_raw(2) = js->l_hand.torque.z;
    r_hand_force_raw(0) = js->r_hand.force.x;
    r_hand_force_raw(1) = js->r_hand.force.y;
    r_hand_force_raw(2) = js->r_hand.force.z;
    r_hand_torque_raw(0) = js->r_hand.torque.x;
    r_hand_torque_raw(1) = js->r_hand.torque.y;
    r_hand_torque_raw(2) = js->r_hand.torque.z;
    forceFilter(l_hand_force_raw, l_hand_force, l_hand_force_buffer, 0.1);
    forceFilter(l_hand_torque_raw, l_hand_torque, l_hand_torque_buffer, 0.1);
    forceFilter(r_hand_force_raw, r_hand_force, r_hand_force_buffer, 0.1);
    forceFilter(r_hand_torque_raw, r_hand_torque, r_hand_torque_buffer, 0.1);
}

Eigen::Matrix3d UpperBodyPlanner::RPY2EigenMat(const Eigen::Vector3d& angle) {
    Eigen::Matrix3d outMat;
    KDL::Rotation convert = KDL::Rotation::RPY(angle(0), angle(1), angle(2));
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            outMat(i,j) = convert.data[3 * i + j];
    return outMat;
}

Eigen::Matrix3d UpperBodyPlanner::tfMat2Eigen3d(const tf::Matrix3x3 &inMat_) {
    Eigen::Matrix3d outMat;
    for (int i = 0; i < 3; i++)
       for (int j = 0; j < 3; j++)
           outMat(i,j) = inMat_[i][j];
    return outMat;
}

Eigen::Vector3d UpperBodyPlanner::tfVec2Eigen3d(const tf::Vector3 &inVec_) {
    Eigen::Vector3d outVec;
    for (int i = 0; i < 3; i++)
        outVec(i) = inVec_[i];
    return outVec;
}

Eigen::VectorXd UpperBodyPlanner::stdVec2EigenXd(const std::vector<double>& inVec_) {
    Eigen::VectorXd outVec(inVec_.size());
    for (int i = 0; i < inVec_.size(); i++) {
        outVec(i) = inVec_[i];
    }
    return outVec;
}

Eigen::Quaterniond UpperBodyPlanner::Rmat2Quaternion(const Eigen::Matrix3d& inMat_) {
    Eigen::Quaterniond outQ;
    KDL::Rotation convert;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            convert.data[3 * i + j] = inMat_(i,j);
        }
    }
    convert.GetQuaternion(outQ.x(), outQ.y(), outQ.z(), outQ.w());
    return outQ;
}

std::vector<double> UpperBodyPlanner::EigenXd2stdVec(const Eigen::VectorXd& inVec_) {
    std::vector<double> outVec(inVec_.size());
    for (int i = 0; i < inVec_.size(); i++) {
        outVec[i] = inVec_(i);
    }
    return outVec;   
}

geometry_msgs::Pose UpperBodyPlanner::Eigen2msgPose(const Eigen::Vector3d& translation, const Eigen::Quaterniond& q) {
    geometry_msgs::Pose out_pose;
    out_pose.position.x = translation(0);
    out_pose.position.y = translation(1);
    out_pose.position.z = translation(2);
    out_pose.orientation.w = q.w();
    out_pose.orientation.x = q.x();
    out_pose.orientation.y = q.y();
    out_pose.orientation.z = q.z();
    return out_pose;
}

void UpperBodyPlanner::msgPose2Eigen(const geometry_msgs::Pose& input, Eigen::Vector3d& translation, Eigen::Quaterniond& q) {
    translation(0) = input.position.x;
    translation(1) = input.position.y;
    translation(2) = input.position.z;
    q.w() = input.orientation.w;
    q.x() = input.orientation.x;
    q.y() = input.orientation.y;
    q.z() = input.orientation.z;
}

void UpperBodyPlanner::msgPose2Eigen(const geometry_msgs::Pose& input, Eigen::Vector3d& translation, Eigen::Matrix3d& rotation) {
    translation(0) = input.position.x;
    translation(1) = input.position.y;
    translation(2) = input.position.z;
    Eigen::Quaterniond convert;
    convert.w() = input.orientation.w;
    convert.x() = input.orientation.x;
    convert.y() = input.orientation.y;
    convert.z() = input.orientation.z;
    rotation = convert.toRotationMatrix();
}

void UpperBodyPlanner::printOutJoints() {
    std::vector<std::string> joints = getJoints();
    if (joints.size() == 0) ROS_ERROR("Can not get joints.");
    else {
        for (int i = 0; i < joints.size(); i++) {
            std::cout << "Joint number is: " << i << "   joint name is: " << joints[i] << std::endl;   
        }
    }
}

void UpperBodyPlanner::printOutVariables() {
    std::vector<std::string> variables_name = kinematic_state->getVariableNames();
    if (variables_name.size() == 0) ROS_ERROR("Can not get variable name.");
    else {
        std::cout << "********************************************************************" << std::endl;
        for (int i = 0; i < variables_name.size(); i++) {
            std::cout << "variable name is: " << variables_name[i] << ",  variable number is: " << i << std::endl;
        }
    }
}

bool UpperBodyPlanner::checkTrajectory(const moveit_msgs::RobotTrajectory& input_trajectory, moveit_msgs::RobotTrajectory& output_trajectory) {
    moveit_msgs::RobotTrajectory check_trajectory = input_trajectory;
    int input_traj_size = input_trajectory.joint_trajectory.points.size();
    if (input_traj_size == 0) {
        ROS_ERROR("No points in the trajectory.");
        singularity_check = true;
        return false;
    }
    else if (input_traj_size == 1) {
        ROS_INFO("1 point in the trajectory.");
        output_trajectory = check_trajectory;
        singularity_check = true;
        return true;
    }
    int i = 0;
    singularity_check = true;
    while (i < (check_trajectory.joint_trajectory.points.size() - 1)) {
        //std::cout << "Checking point number is: " << i << std::endl; 
        Eigen::VectorXd front = stdVec2EigenXd(check_trajectory.joint_trajectory.points[i].positions);
        Eigen::VectorXd back = stdVec2EigenXd(check_trajectory.joint_trajectory.points[i + 1].positions);
        Eigen::VectorXd difference = front - back;
        if (difference.norm() == 0) {
            //std::cout << "The point " << i << " and " << i + 1 << " is too close." << std::endl;
            //std::cout << "Erase point " << i + 1 << " points" << std::endl;
            check_trajectory.joint_trajectory.points.erase(check_trajectory.joint_trajectory.points.begin() + (i + 1));
        }
        else {
            if (difference.norm() >= 0.5) {
                ROS_WARN("Hit singularity");
                singularity_check = false;
            }
            i++;
        }  
    }
    //double fix_time = check_trajectory.joint_trajectory.points[0].time_from_start.toSec();
    double fix_time = path_step / move_velocity;
    Eigen::VectorXd velocity_cmd = Eigen::VectorXd::Zero(7); 
    double start_t = 0;
    for (int j = 0; j < check_trajectory.joint_trajectory.points.size(); j++) {
        if (j == 0) {
            start_t = start_t + 2 * fix_time;
            velocity_cmd = Eigen::VectorXd::Zero(7);
        }
        else if (j == (check_trajectory.joint_trajectory.points.size() - 1)) {
            start_t = start_t + 2 * fix_time;
            velocity_cmd = Eigen::VectorXd::Zero(7);
        }
        else {
            Eigen::VectorXd current_joint_angle = stdVec2EigenXd(check_trajectory.joint_trajectory.points[j + 1].positions);
            Eigen::VectorXd last_joint_angle = stdVec2EigenXd(check_trajectory.joint_trajectory.points[j].positions);
            velocity_cmd = (current_joint_angle - last_joint_angle) / fix_time;
        }
            start_t = start_t + fix_time;
        ros::Duration start_time((start_t));
        check_trajectory.joint_trajectory.points[j].time_from_start = start_time;
        check_trajectory.joint_trajectory.points[j].velocities = EigenXd2stdVec(velocity_cmd);
    }
    output_trajectory = check_trajectory;
    return true;
}

void UpperBodyPlanner::printOutQuatern(const Eigen::Quaterniond& q) {
    std::cout << q.w() << std::endl;
    std::cout << q.x() << std::endl;
    std::cout << q.y() << std::endl;
    std::cout << q.z() << std::endl;
}

void UpperBodyPlanner::forceFilter(const Eigen::Vector3d& input, Eigen::Vector3d& output, Eigen::Vector3d& buffer, const double &factor) {
    Eigen::Vector3d difference = buffer - input;
    if (difference.norm() >= filter_threshold) output = input;
    else output = factor * input + (1 - factor) * buffer;
    buffer = output;
}