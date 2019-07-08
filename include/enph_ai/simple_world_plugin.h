#ifndef WORLD_PLUGIN_TUTORIAL_HH
#define WORLD_PLUGIN_TUTORIAL_HH

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{

class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial();

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  void Update();
};

}  // namespace gazebo

#endif
