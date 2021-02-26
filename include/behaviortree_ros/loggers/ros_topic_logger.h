#ifndef BT_ROSTOPIC_LOGGER_H
#define BT_ROSTOPIC_LOGGER_H

#include <vector>
#include <memory>
#include <utility>
#include "ros/ros.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "behaviortree_ros/StatusChangeLog.h"


namespace BT
{

class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  RosTopicLogger(ros::NodeHandle &nh, const BT::Tree & tree)
    : StatusChangeLogger(tree.rootNode()), node_(nh)
  {
    publisher_ = node_.advertise<behaviortree_ros::StatusChangeLog>("behavior_tree_log", 10);
    writeBT(tree);
  }

  void callback(
      Duration timestamp,
      const TreeNode & node,
      NodeStatus prev_status,
      NodeStatus status) override
  {

    behaviortree_ros::StatusChange event;
    behaviortree_ros::NodeStatus ros_status;

    int32_t t_sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp).count();
    int32_t t_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp).count();
    event.timestamp = ros::Time(t_sec, t_nsec);

    ros_status.value = this->toROS(status);
    event.status = ros_status;
    ros_status.value = this->toROS(prev_status);
    event.prev_status = ros_status;

    event.uid = node.UID();
    event_log_.state_changes.push_back(event);

  }

  void writeBT(const BT::Tree &tree){
    event_log_.behavior_tree = this->BTtoROS(tree);
  }

  void flush() override
  {
    if(event_log_.state_changes.empty())
      return;

    publisher_.publish(event_log_);
    event_log_.state_changes.clear();
  }

protected:
  ros::NodeHandle& node_;
  ros::Publisher publisher_;
  behaviortree_ros::StatusChangeLog event_log_;


  behaviortree_ros::BehaviorTree BTtoROS(const BT::Tree & tree){
    behaviortree_ros::BehaviorTree ros_tree;
    behaviortree_ros::TreeNode ros_node;

    ros_tree.root_uid = tree.rootNode()->UID();

    for (const auto &node : tree.nodes){
      ros_node.uid = node->UID();
      ros_node.type = this->toROS(node->type());
      //ros_node.params = ...
      //ros_node.status = this->toROS(node->status());
      //ros_node.children_uid = ...
      ros_node.instance_name = node->name();
      ros_node.registration_name = node->registrationName();
      ros_tree.nodes.push_back(ros_node);
    }
    return ros_tree;
  }

  int8_t toROS(NodeType type){
    switch (type)
    {
    case NodeType::ACTION:
      return behaviortree_ros::TreeNode::ACTION;
    case NodeType::CONDITION:
      return behaviortree_ros::TreeNode::CONDITION;
    case NodeType::DECORATOR:
      return behaviortree_ros::TreeNode::DECORATOR;
    case NodeType::CONTROL:
      return behaviortree_ros::TreeNode::CONTROL;
    case NodeType::SUBTREE:
      return behaviortree_ros::TreeNode::SUBTREE;
    default:
      return behaviortree_ros::TreeNode::UNDEFINED;
    }
  }

  int8_t toROS(NodeStatus status){
    switch (status)
    {
    case NodeStatus::IDLE:
      return behaviortree_ros::NodeStatus::IDLE;
    case NodeStatus::RUNNING:
      return behaviortree_ros::NodeStatus::RUNNING;
    case NodeStatus::FAILURE:
      return behaviortree_ros::NodeStatus::FAILURE;
    default:
      return behaviortree_ros::NodeStatus::IDLE;
    }
  }

};

}

#endif   // BT_ROSTOPIC_LOGGER_H
