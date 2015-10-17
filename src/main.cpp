#include <math.h> 
#include "hydra_move_group.h"
#include <Eigen/Geometry>

Eigen::Affine3d target;

Eigen::Vector3d translation;

Eigen::Matrix3d rotation;

std::vector<geometry_msgs::Pose> pose_sequence;

Eigen::Affine3d target1;

Eigen::Vector3d translation1;

Eigen::Matrix3d rotation1;

std::vector<geometry_msgs::Pose> pose_sequence1;

bool button = false;
bool button1 = false;
//bool key = false;

void myCallback(const interaction_cursor_msgs::InteractionCursorUpdate poseSubscribed) {
    // std::cout<<" i am in mycallback "<<std::endl;

    target = Eigen::Affine3d::Identity();



    Eigen::Quaterniond q(poseSubscribed.pose.pose.orientation.w,
            poseSubscribed.pose.pose.orientation.x,
            poseSubscribed.pose.pose.orientation.y,
            poseSubscribed.pose.pose.orientation.z);

    translation << poseSubscribed.pose.pose.position.x,
            poseSubscribed.pose.pose.position.y,
            poseSubscribed.pose.pose.position.z;
    std::cout << " 2 " << std::endl;
    rotation = q.toRotationMatrix();
    target.translation() = translation;
    target.linear() = rotation;




    std::cout << "please move hydra to a proper place and click the button. " << std::endl;

    if (poseSubscribed.button_state == 1) {
        button = true;

    }
}

void myCallback1(const interaction_cursor_msgs::InteractionCursorUpdate poseSubscribed) {
    // std::cout<<" i am in mycallback "<<std::endl;

    target1 = Eigen::Affine3d::Identity();



    Eigen::Quaterniond q(poseSubscribed.pose.pose.orientation.w,
            poseSubscribed.pose.pose.orientation.x,
            poseSubscribed.pose.pose.orientation.y,
            poseSubscribed.pose.pose.orientation.z);

    translation1 << poseSubscribed.pose.pose.position.x,
            poseSubscribed.pose.pose.position.y,
            poseSubscribed.pose.pose.position.z;
    std::cout << " 3 " << std::endl;
    rotation1 = q.toRotationMatrix();
    target1.translation() = translation1;
    target1.linear() = rotation1;




    std::cout << "please move hydra to a proper place and click the button. " << std::endl;

    if (poseSubscribed.button_state == 1) {
        button1 = true;

    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo");
    ros::start();
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    std::cout << "i am in demo~" << std::endl;
    ros::NodeHandle nh;
    ros::Subscriber mySubscriber;
    ros::Subscriber mySubscriber1;
    
    mySubscriber = nh.subscribe<interaction_cursor_msgs::InteractionCursorUpdate>("/interaction_cursor_left/update", 1, myCallback);
    mySubscriber1 = nh.subscribe<interaction_cursor_msgs::InteractionCursorUpdate>("/interaction_cursor_right/update", 1, myCallback1);
    
    move_group_interface::MoveGroup::Plan l_plan;
    move_group_interface::MoveGroup::Plan r_plan;
    

    UpperBodyPlanner left_group("left_arm");
   
    left_group.setMovementVelocity(0.05);
    

    UpperBodyPlanner right_group("right_arm");
   
    right_group.setMovementVelocity(0.05);

    int number = 0;
    while (!left_group.checkConstructorStatus("l_palm")||!right_group.checkConstructorStatus("right_palm")) {
        ROS_INFO("Waiting for atlas state and tf to be ready...");
        number++;
        if (number >= 10) {
            ROS_ERROR("Not ready.");
            break;
        }
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    std::cout << " 4 " << std::endl;
    ros::Rate r(100);
    while (true) {
        ros::spinOnce();
        left_group.updateRobotState();
        right_group.updateRobotState();
        if (button) {
            button = false;

            left_group.pose_moveTo("l_palm", target, 5, pose_sequence);
            left_group.previewPathPlan(pose_sequence, l_plan);
        
            left_group.execute(l_plan);
        }
        if (button1) {
            button1 = false;

            right_group.pose_moveTo("right_palm", target1, 5, pose_sequence1);
            right_group.previewPathPlan(pose_sequence1, r_plan);
        
            right_group.execute(r_plan);
        }
        r.sleep();
    }
    

    return 0;
}

