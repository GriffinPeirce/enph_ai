#include <enph_ai/collision_plugin.h>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(CollisionPlugin);

CollisionPlugin::CollisionPlugin()
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

  ROS_INFO("Hello World!");
}

}  // namespace gazebo
