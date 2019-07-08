#include <enph_ai/simple_world_plugin.h>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);

WorldPluginTutorial::WorldPluginTutorial()
{
}

void WorldPluginTutorial::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
    << "Load the Gazebo system plugin 'libenph_ai.so' in the enph_ai package)");
    return;
  }

  ROS_INFO("Hello World!");
}


void WorldPluginTutorial::Update()
{
  ROS_INFO("Update()");
}


}  // namespace gazebo
