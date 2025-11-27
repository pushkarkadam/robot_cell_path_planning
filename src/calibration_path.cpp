#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    // Initialise ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "calibration_path",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("calibration_path");

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

    /*
    - ur10e_shoulder_lift_joint
    - ur10e_wrist_1_joint
    - ur10e_wrist_3_joint
    - ur10e_wrist_2_joint
    - ur10e_shoulder_pan_joint
    - ur10e_elbow_joint
    position:
    - -2.086670061151022
    - 0.4681543546864013
    - 3.263265609741211
    - 1.383528709411621
    - 5.837290287017822
    - -2.046292543411255
    */


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

    

    // float eval_point_x[] = {0.825,1.05562076,1.15968868,0.9292113,0.87891276,1.04329609,1.10618927,0.94121135,0.93252655,1.03143218,1.05226851,0.95332299};
    // float eval_point_y[] = {-0.473,-0.32697376,-0.4914744,-0.63733321,-0.48489291,-0.3806577,-0.47935454,-0.58374082,-0.49698328,-0.43454754,-0.4671709,-0.5300323};
    // float eval_point_z[] = {0.013,0.013,0.00991604,0.01400203,0.01192913,0.01139528,0.00912942,0.0130812,0.01247212,0.01025719,0.00992753,0.01241558,};

    
    // case 1
    // float eval_point_x[] = {0.825, 1.056, 1.16, 0.929, 0.879, 1.044, 1.106, 0.941, 0.933, 1.032, 1.052, 0.953};
    // float eval_point_y[] = {-0.473, -0.327, -0.492, -0.637, -0.485, -0.38, -0.479, -0.584, -0.497, -0.435, -0.467, -0.53};
    // float eval_point_z[] = {0.013, 0.013, 0.013, 0.014, 0.013, 0.012, 0.014, 0.013, 0.013, 0.013, 0.012, 0.014};

    // case 2
    // float eval_point_x[] = {1.001, 1.156, 0.995, 0.84, 0.991, 1.101, 1.005, 0.894, 0.98, 1.047, 1.015, 0.949};
    // float eval_point_y[] = {-0.326, -0.551, -0.661, -0.436, -0.38, -0.541, -0.607, -0.446, -0.434, -0.531, -0.553, -0.456};
    // float eval_point_z[] = {0.012, 0.013, 0.015, 0.012, 0.012, 0.013, 0.013, 0.013, 0.013, 0.013, 0.014, 0.013};

    // case 3
    float eval_point_x[] = {0.832, 1.069, 1.161, 0.924, 0.884, 1.054, 1.109, 0.94, 0.937, 1.038, 1.056, 0.955};
    float eval_point_y[] = {-0.485, -0.348, -0.506, -0.645, -0.498, -0.399, -0.494, -0.592, -0.51, -0.45, -0.482, -0.541};
    float eval_point_z[] = {0.034, 0.033, 0.103, 0.099, 0.046, 0.045, 0.088, 0.088, 0.06, 0.059, 0.076, 0.074};

    // float x_init = eval_point_x[0];


    auto const initial_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = -0.7071068;
        msg.orientation.y = 0;
        msg.orientation.z = 0.7071068;
        msg.orientation.w = 0;
        msg.position.x = -0.825 + 0.020;
        msg.position.y = 0.473 + 0.020;
        msg.position.z = 0.013 + 0.030;
        return msg;
    }();

    waypoints.push_back(initial_pose);

    // geometry_msgs::msg::Pose pose = initial_pose;
    geometry_msgs::msg::Pose eval_point;

    for (int i = 0; i < 12; i++) {
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