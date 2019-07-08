#ifndef COLLISION_PLUGIN_HH
#define COLLISION_PLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{

class CollisionPlugin : public WorldPlugin
{
public:
  /// Constructor.
  CollisionPlugin();

  /// Documentation inherited.
  /// Called when a Plugin is first created, and after the World has been
  /// loaded. This function should not be blocking.
  /// \param[in] _world Pointer the World
  /// \param[in] _sdf Pointer the the SDF element of the plugin.
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

};

}  // namespace gazebo

#endif
