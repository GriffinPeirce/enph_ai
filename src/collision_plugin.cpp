#include <enph_ai/collision_plugin.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>

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

  ROS_INFO("Initializing collision plugin!");
  this->collisionNode->Init();
  this->collisionSub = this->collisionNode->Subscribe("/gazebo/default/physics/contacts",
                                          &CollisionPlugin::OnCollisionMsg, this);
}

void CollisionPlugin::OnCollisionMsg(ConstContactsPtr &_contacts)
{
  ROS_INFO("On collision msg");
  for (unsigned int i = 0; i < _contacts->contact_size(); ++i) {
    std::string collisionStr1 = _contacts->contact(i).collision1();
    std::string collisionStr2 = _contacts->contact(i).collision2();
    ROS_INFO_STREAM("Collision between " << collisionStr1 << " and " << collisionStr2);
  }
}

}  // namespace gazebo
