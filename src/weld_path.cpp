#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "robot_cell_path_planning/read_data.h"

int main(int argc, char** argv)
{

    if (argc != 2) {
        std::cerr << "Usage:  ros2 robot_cell_path_planning weld_path /home/<user>/Document/file.csv" << std::endl;
    }

    // Extracting CSV file
    std::string fileName = argv[1];

    // Initialise ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "weld_path",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("weld_path");

    // We spin up a SingleThreadedExecutor for the current state monitor to get
    // information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){
        executor.spin();
    });

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_arm");

    move_group_interface.setPlanningTime(60.0);

    // Construct and initialise MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node,
        "ur10e_base_link",
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()
    };

    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Create a closure for updating the text in rviz
    auto const draw_title = [&moveit_visual_tools](auto text){
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0; // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    };

    auto const prompt = [&moveit_visual_tools](auto text){
        moveit_visual_tools.prompt(text);
    };

    auto const draw_trajectory_tool_path = 
        [&moveit_visual_tools, 
            jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_arm"),
            tool_link = move_group_interface.getRobotModel()->getLinkModel("ur10e_tip")](auto const trajectory) {
                moveit_visual_tools.publishTrajectoryLine(trajectory, tool_link, jmg);
            };

    // Get the names of the joints
    const std::vector<std::string>& joint_names = move_group_interface.getJointNames();

    RCLCPP_INFO(logger, "Joint group has %zu joints", joint_names.size());

    for (const auto& name : joint_names) {
        RCLCPP_INFO(logger, "Joint: %s", name.c_str());
    }

    // Map of joint names to values
    std::map<std::string, double> joint_map = {
        {"ur10e_shoulder_pan_joint", 5.837},
        {"ur10e_shoulder_lift_joint", -2.086},
        {"ur10e_elbow_joint", -2.04},
        {"ur10e_wrist_1_joint", 0.468},
        {"ur10e_wrist_2_joint", 1.383},
        {"ur10e_wrist_3_joint", 3.26}
    };

    // Convert to ordered vector
    std::vector<double> joint_goal;
    for (const auto& name : joint_names) {
        joint_goal.push_back(joint_map[name]);
    }

    // Set joint target
    move_group_interface.setJointValueTarget(joint_goal);


    // Adding floor collision
    auto const floor = [frame_id = move_group_interface.getPlanningFrame()]{
        moveit_msgs::msg::CollisionObject floor;
        floor.header.frame_id = frame_id;
        floor.id = "floor";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 5.0;
        primitive.dimensions[primitive.BOX_Y] = 5.0;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose floor_pose;
        floor_pose.orientation.w = 1.0;
        floor_pose.orientation.x = 0.0;
        floor_pose.orientation.y = 0.0;
        floor_pose.orientation.z = -0.005;

        floor.primitives.push_back(primitive);
        floor.primitive_poses.push_back(floor_pose);
        floor.operation = floor.ADD;

        return floor;
    }();

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(floor);

    // Create a plan to that target pose 
    prompt("Press 'next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success){
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
    } else {
        draw_title("Planning Failed");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Creating a cartesian path

    // List of waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;


    std::vector<std::string> columnNames = {"x", "y", "z"};

    std::map<std::string, std::vector<double>> table = readViaPoints(fileName, columnNames);

    std::vector<double> eval_point_x = table["x"];
    std::vector<double> eval_point_y = table["y"];
    std::vector<double> eval_point_z = table["z"];

    double x0 = eval_point_x[0];
    double y0 = eval_point_y[0];
    double z0 = eval_point_z[0];


    auto const initial_pose = [x0, y0, z0]{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = -0.7071068;
        msg.orientation.y = 0;
        msg.orientation.z = 0.7071068;
        msg.orientation.w = 0;
        msg.position.x = -x0 + 0.020;
        msg.position.y = -y0 + 0.020;
        msg.position.z = z0 + 0.030;
        return msg;
    }();

    waypoints.push_back(initial_pose);

    // geometry_msgs::msg::Pose pose = initial_pose;
    geometry_msgs::msg::Pose eval_point;

    for (int i = 0; i < 2; i++) {
        eval_point.position.x = -eval_point_x[i]; // Negative signs transpose from 'base' to 'base_link'
        eval_point.position.y = -eval_point_y[i];
        eval_point.position.z = eval_point_z[i] + 0.010;
        eval_point.orientation.x = -0.7071068;
        eval_point.orientation.y = 0;
        eval_point.orientation.z = 0.7071068;
        eval_point.orientation.w = 0;

        waypoints.push_back(eval_point);

        RCLCPP_INFO(logger, "Waypoint %d: x %.4f, y %.4f, z %.4f", i, eval_point.position.x, eval_point.position.y, eval_point.position.z);
    }

    // Prepare output trajectory
    moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

    double eef_step = 0.005; 
    double jump_threshold = 0.0;

    moveit_msgs::msg::MoveItErrorCodes error_code;

    double fraction = move_group_interface.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        cartesian_trajectory
    );

    if (fraction < 1.0) {
        RCLCPP_WARN(logger, "Only %f%% of the path was achieved", fraction * 100.0);
    }


    // Create a plan to that target pose 
    prompt("Press 'next' in the RvizVisualToolsGui window to plan");
    draw_title("Point_Move");
    moveit_visual_tools.trigger();

    for (size_t i = 0; i < waypoints.size(); i++) {
        moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i));
    }
    moveit_visual_tools.trigger();

    // Execute the plan
    if (fraction > 0.0){
        draw_trajectory_tool_path(cartesian_trajectory);
        moveit_visual_tools.trigger();
        prompt("Press 'next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(cartesian_trajectory);
    } else {
        draw_title("Planning Failed");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}