#include "behaviortree_ros2/bt_topic_pub_node_async.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;
using String = std_msgs::msg::String;

class PublishString : public AsyncRosTopicPubNode<String>
{

public:
  PublishString(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicPubNode<String>(name, conf, params) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<std::string>("data", "String message")
    });
  }

  bool setMessage(String& msg) override
  {
    getInput("data", msg.data);
    return true;
  }
};