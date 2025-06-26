#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <cstdlib>
#include <geometry_msgs/msg/pose.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_bt_utils/condition_topic_subscriber.hpp>

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ros2_bt_utils/action_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using NavigateToPoseFeedback = typename rclcpp_action::ClientGoalHandle<
    nav2_msgs::action::NavigateToPose>::Feedback::ConstSharedPtr;

class ActionGoto : public ros2_bt_utils::ActionROSActionClient<nav2_msgs::action::NavigateToPose>
{
public:
    ActionGoto(const std::string& name, const BT::NodeConfiguration& config)
    : ros2_bt_utils::ActionROSActionClient<nav2_msgs::action::NavigateToPose>(name, config, "navigate_to_pose")
    {
        RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::Pose>("pose")
        };
    }

private:
    nav2_msgs::action::NavigateToPose::Goal computeGoal() override
    {
        RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Computing new goal");
        geometry_msgs::msg::Pose goal_pose;
        getInput("pose", goal_pose);  

        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.pose = goal_pose;

        return goal;
    }

    void elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback) override
    {
        RCLCPP_INFO(
            ros2_bt_utils::ROSNode()->get_logger(),
            "ETA: %d",
            feedback.get()->estimated_time_remaining.sec);
    }

    BT::NodeStatus elaborateResultAndReturn(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) override
    {   
        RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Result Code %i.", result.code);
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Navigation succeeded.");
                return BT::NodeStatus::SUCCESS;

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(ros2_bt_utils::ROSNode()->get_logger(), "Navigation aborted.");
                return BT::NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(ros2_bt_utils::ROSNode()->get_logger(), "Navigation canceled.");
                return BT::NodeStatus::FAILURE;

            default:
                RCLCPP_ERROR(ros2_bt_utils::ROSNode()->get_logger(), "Unknown result code.");
                return BT::NodeStatus::FAILURE;
        }
    }
};


class TestIterationMonitor : public BT::SyncActionNode
{
public:
    TestIterationMonitor(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
          BT::InputPort<int>("total_tests")
        };
    }

    BT::NodeStatus tick() override
    {
        auto total_tests = getInput<int>("total_tests").value();
        if (total_tests > 0)
        {
            // RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Test Count %i", total_tests);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(ros2_bt_utils::ROSNode()->get_logger(), "[TestIterationMonitor] Returned Failure");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class IncrementCounter : public BT::SyncActionNode
{
public:
    IncrementCounter(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
          BT::InputPort<std::string>("key"),
          BT::InputPort<int>("start_value")
        };
    }

    BT::NodeStatus tick() override
    {
        auto key = getInput<std::string>("key").value();
        int start_value = getInput<int>("start_value").value();

        int count = start_value;
        if (config().blackboard->get(key, count))
        {
            count -= 1;
        }
        else
        {
            count = start_value;
        }

        config().blackboard->set(key, count);

        std::cout << "[IncrementCounter] " << key << " = " << count << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

class StartRosbagRecord : public BT::SyncActionNode
{
public:
    StartRosbagRecord(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("topics"), 
            BT::InputPort<std::string>("bag_path")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string topics;
        if (!getInput("topics", topics))
        {
            std::cerr << "Missing required input [topics]" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::string bag_path;
        if (!getInput("bag_path", bag_path))
        {
            std::cerr << "Missing required input [bag_path]" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::string command = "ros2 bag record " + topics + " -o " + bag_path + " &";
        int ret = std::system(command.c_str());
        if (ret == 0)
        {
            std::cout << "Started rosbag recording." << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cerr << "Failed to start rosbag recording." << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
};

class StopRosbagRecord : public BT::SyncActionNode
{
public:
    StopRosbagRecord(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        int ret = std::system("pkill -f 'ros2 bag record'");
        if (ret == 0)
        {
            std::cout << "Stopped rosbag recording." << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cerr << "Failed to stop rosbag recording." << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
};

class KillRosNodes : public BT::SyncActionNode
{
public:
    KillRosNodes(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        int ret = std::system("ps aux | grep -iE 'nav2_bringup|test_automation_node|rviz2|component_container_isolated' | grep -v grep | awk '{ print $2; }' | xargs kill -${2:-'TERM'}");
        if (ret == 0)
        {
            std::cout << "Shutting Down" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cerr << "Failed to Shutdown" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
};

class WaitForTopics : public BT::AsyncActionNode
{
public:
    WaitForTopics(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        std::string topic1_name;
        if (!getInput("topic1_name", topic1_name))
        {
            throw BT::RuntimeError("Missing required input [topic1_name]");
        }

        // std::string topic2_name;
        // if (!getInput("topic2_name", topic2_name))
        // {
        //     throw BT::RuntimeError("Missing required input [topic2_name]");
        // }        

        node_ = rclcpp::Node::make_shared("wait_for_topics_node");

        sub1_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic1_name, 10, [this](const nav_msgs::msg::Odometry::SharedPtr) {
                // RCLCPP_INFO(node_->get_logger(), "Message Recieved");
                received_topic1_ = true;
            });
        RCLCPP_INFO(node_->get_logger(), "Node Created");

        // sub2_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     topic2_name, 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
        //         RCLCPP_INFO(node_->get_logger(), "Message Recieved");
        //         received_topic2_ = true;
        //     });

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });
    }

    ~WaitForTopics()
    {
        executor_->cancel();
        spin_thread_.join();
    }

    BT::NodeStatus tick() override
    {
        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(5);

        // while (!received_topic1_ || !received_topic2_)
        while (!received_topic1_)
        {
            RCLCPP_INFO(node_->get_logger(), "Waiting for all topics...");
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                RCLCPP_WARN(node_->get_logger(), "Timeout waiting for topic");
                return BT::NodeStatus::FAILURE;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(node_->get_logger(), "Received topics");
        return BT::NodeStatus::SUCCESS;
    }

    void halt() override
    {
        // started_ = false;
        received_topic1_ = false;
        // received_topic2_ = false;
    }

    static BT::PortsList providedPorts() 
    { 
        return 
        { 
            BT::InputPort<std::string>("topic1_name"), 
            // BT::InputPort<std::string>("topic2_name") 
        }; 
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub2_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;

    std::atomic<bool> received_topic1_{false};
    // std::atomic<bool> received_topic2_{false};

    // std::chrono::steady_clock::time_point start_time_;
    // std::chrono::steady_clock::time_point now_;
    // std::atomic<bool> started_;
    std::chrono::time_point<std::chrono::steady_clock> start;
    std::chrono::seconds timeout;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = ros2_bt_utils::ROSNode();
//   node->declare_parameter("battery", 1.0);
  node->declare_parameter(
    "bt_filename",
    std::string(
      ament_index_cpp::get_package_share_directory("test_automation_pkg") + "/bt_xml/straight_line_test.xml"));
  rclcpp::Parameter param_bt_filename = node->get_parameter("bt_filename");

  RCLCPP_INFO(
    node->get_logger(), "Loading  BT XML filename : %s",
    param_bt_filename.value_to_string().c_str());

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ActionGoto>("GoTo");
//   factory.registerNodeType<ActionWait>("Wait");

//   factory.registerNodeType<ConditionBatteryLevelAbove>("BatteryLevelAbove");
//   factory.registerNodeType<ConditionBatteryIsNotCharging>("BatteryIsNotCharging");
  factory.registerNodeType<TestIterationMonitor>("TestIterationMonitor");
//   factory.registerNodeType<ConditionIsAt>("IsAtPose");
  factory.registerNodeType<IncrementCounter>("IncrementCounter");
  factory.registerNodeType<StartRosbagRecord>("StartRosbagRecord");
  factory.registerNodeType<StopRosbagRecord>("StopRosbagRecord");
  factory.registerNodeType<KillRosNodes>("KillRosNodes");
  factory.registerNodeType<WaitForTopics>("WaitForTopics");

  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromFile(param_bt_filename.value_to_string().c_str(), blackboard);

  geometry_msgs::msg::Pose charging_station_pose;
  charging_station_pose.position.x = 1.1398714780807495;
  charging_station_pose.position.y = 0.5856299996376038;

  charging_station_pose.orientation.z = 0.7002860307693481;
  charging_station_pose.orientation.w = 0.7138128280639648;

  blackboard->set("charging_station", charging_station_pose);

  geometry_msgs::msg::Pose pose_of_interest1;
  pose_of_interest1.position.x = 2.5840141773223877;
  pose_of_interest1.position.y = -1.8378535509109497;

  pose_of_interest1.orientation.z = 0.918582558631897;
  pose_of_interest1.orientation.w = 0.39520755410194397;

  blackboard->set("pose_of_interest1", pose_of_interest1);

  geometry_msgs::msg::Pose pose_of_interest2;
  pose_of_interest2.position.x = -1.6733359098434448;
  pose_of_interest2.position.y = 2.099357843399048;

  pose_of_interest2.orientation.z = -0.3917270004749298;
  pose_of_interest2.orientation.w = 0.9199512600898743;

  blackboard->set("pose_of_interest2", pose_of_interest2);

//   geometry_msgs::msg::Pose pose_of_interest3;
//   pose_of_interest2.position.x = 4.0;
//   pose_of_interest2.position.y = 5.0;

//   pose_of_interest2.orientation.z = 0.707;
//   pose_of_interest2.orientation.w = 0.707;

//   blackboard->set("pose_of_interest3", pose_of_interest3);

  tree.rootBlackboard()->set("total_tests", 2);


// BT::StdCoutLogger logger_cout(tree);
  #ifdef ZMQ_FOUND
    // This publish tree to visualize it with Groot
    BT::PublisherZMQ publisher_zmq(tree);
  #endif


  BT::FileLogger logger_file(tree, "bt_trace.fbl");

  while (rclcpp::ok()) {
    tree.rootNode()->executeTick();
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  return 0;
}