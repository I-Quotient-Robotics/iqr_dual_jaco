#include <ros/ros.h>
#include <string>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/ArmPoseAction.h>

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kinova_msgs/ArmPoseAction.h>

#include <actionlib/client/simple_action_client.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> PoseClient;

class PickTaskAction
{
public:

  ros::NodeHandle nh_;
  PoseClient pc_left_, pc_right_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface robot_group;

  PickTaskAction(std::string name) : 
    robot_group("robot_group"),
    pc_left_("left_arm_driver/pose_action/tool_pose", true),
    pc_right_("right_arm_driver/pose_action/tool_pose", true)
  {
    robot_group.setPlanningTime(5.0);
  }

  ~PickTaskAction(void)
  {
  }

  void ExcuteThread() {

    ros::AsyncSpinner spinner(3);
    spinner.start();

    kinova_msgs::ArmPoseGoal eff_pose;
    // 末端沿ｚ轴方向前移3cm
    eff_pose.pose.header.frame_id = "right_arm_end_effector";

    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = 0.03;
    eff_pose.pose.pose.position.z = 0.05;

    eff_pose.pose.pose.position.z = 0.05;
    eff_pose.pose.pose.orientation.w = 1.0;
    pc_right_.sendGoal(eff_pose);
    pc_right_.waitForResult(ros::Duration(0.0));
    ros::WallDuration(1.0).sleep();
    // 末端沿ｚ轴方向后移５cm
    eff_pose.pose.header.frame_id = "right_arm_end_effector";
    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = -0.03;
    eff_pose.pose.pose.position.z = -0.05;
    eff_pose.pose.pose.position.z = -0.05;
    eff_pose.pose.pose.orientation.w = 1.0;
    pc_right_.sendGoal(eff_pose);
    pc_right_.waitForResult(ros::Duration(0.0));
    ros::WallDuration(1.0).sleep();

    return;
  }

  void SetArmPose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double>& joint_value) {
    move_group.setJointValueTarget(joint_value);
    move_group.move();
  }

  void SetGripper(moveit::planning_interface::MoveGroupInterface& move_group, double value) {
    std::vector<double> gripper_pose(2);

    gripper_pose[0] = value;
    gripper_pose[1] = value;
    // gripper_pose[2] = value;

    move_group.setJointValueTarget(gripper_pose);
    move_group.move();
  }

  void moveit() {

    ros::AsyncSpinner spinner(3);
    spinner.start();


  
    // 从config.yaml参数表中读取预设关节位置
    std::vector<double> arm_standby_pose_right(12);
    nh_.getParam("arm_pose/home", arm_standby_pose_right);
    std::vector<double> arm_place_pose_right(12);
    nh_.getParam("arm_pose/pick", arm_place_pose_right);

    //　收回到standby位置
    ROS_INFO("joint control standby");
    robot_group.setJointValueTarget(arm_standby_pose_right);
    robot_group.move();
    ros::WallDuration(1.0).sleep();

    //　抓取位置
    ROS_INFO("joint control pick");
    robot_group.setJointValueTarget(arm_place_pose_right);
    robot_group.move();
    ros::WallDuration(1.0).sleep();

    // 开启新线程，独立完成右臂的位置控制
    std::thread thread(&PickTaskAction::ExcuteThread, this);
    thread.detach();

    kinova_msgs::ArmPoseGoal eff_pose;
    // 末端沿ｚ轴方向前移3cm
    eff_pose.pose.header.frame_id = "left_arm_end_effector";


    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = 0.03;
    eff_pose.pose.pose.orientation.w = 1.0;
    pc_left_.sendGoal(eff_pose);
    pc_left_.waitForResult(ros::Duration(0.0));
    ros::WallDuration(1.0).sleep();

    // 末端沿ｚ轴方向后移3cm
    eff_pose.pose.header.frame_id = "left_arm_end_effector";
    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = 0.0;
    eff_pose.pose.pose.position.z = -0.03;
    eff_pose.pose.pose.orientation.w = 1.0;
    pc_left_.sendGoal(eff_pose);
    pc_left_.waitForResult(ros::Duration(0.0));
    ros::WallDuration(1.0).sleep();

    //　收回到standby位置
    ROS_INFO("joint control standby");
    robot_group.setJointValueTarget(arm_standby_pose_right);
    robot_group.move();
    ros::WallDuration(1.0).sleep();

    return;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_control_node");
  ros::NodeHandle nh;
  PickTaskAction pick("pick_task");
  pick.moveit();
  ros::waitForShutdown();
  return 0;
}