#include "behaviortree_ros2/bt_topic_sub_node_async.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;
using namespace std;
using String = std_msgs::msg::String;

class GetStringFromTopic : public AsyncRosTopicSubNode<String>
{
public:
  GetStringFromTopic(const string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicSubNode<String>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({
      OutputPort<string>("data", "Data received from the topic (string)")
    });
  }

  NodeStatus onTick(const std::shared_ptr<String>& msg) override
  {
    if (!msg)
    {
      return NodeStatus::RUNNING;
    }

    setOutput("data", msg->data);
    return NodeStatus::SUCCESS;
  }

  bool latchLastMessage() const override
  {
    return true;
  }
};