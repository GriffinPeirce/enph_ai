#include <enph_ai/collision_plugin.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo
{

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(CollisionPlugin);

CollisionPlugin::CollisionPlugin() : WorldPlugin(), collisionNode(new transport::Node())
{
}

void CollisionPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
    << "Load the Gazebo system plugin 'libcollision_plugin.so' in the enph_ai package)");
    return;
  }

  // Setup collision node and sub
  ROS_INFO("Initializing collision plugin!");
  this->collisionNode->Init();
  this->collisionSub = this->collisionNode->Subscribe("/gazebo/default/physics/contacts",
                                          &CollisionPlugin::OnCollisionMsg, this);

  // Initialize ROS transport.
  this->rosNode.reset(new ros::NodeHandle());
  this->contactPub = this->rosNode->advertise<std_msgs::Bool>("/isHit", 100);
}

void CollisionPlugin::OnCollisionMsg(ConstContactsPtr &_contacts)
{
  bool isHit = false;
  // Check each collision. If it involves robot and wall, then it is hit
  for (unsigned int i = 0; i < _contacts->contact_size(); ++i) {
    std::string collisionStr1 = _contacts->contact(i).collision1();
    std::string collisionStr2 = _contacts->contact(i).collision2();
    isHit = isHit || ((collisionStr1.find("robot") != std::string::npos && collisionStr2.find("Walls") != std::string::npos) ||
                      (collisionStr1.find("Walls") != std::string::npos && collisionStr2.find("robot") != std::string::npos));
  }

  // publish a ROS MSG
  this->contactMsg.data = isHit;
  this->contactPub.publish(this->contactMsg);
}

}  // namespace gazebo
