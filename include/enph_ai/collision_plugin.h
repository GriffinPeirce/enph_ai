#ifndef COLLISION_PLUGIN_HH
#define COLLISION_PLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>

namespace gazebo
{

class CollisionPlugin : public WorldPlugin
{
public:
  // Methods

  /// Constructor.
  CollisionPlugin();

  /// Documentation inherited.
  /// Called when a Plugin is first created, and after the World has been
  /// loaded. This function should not be blocking.
  /// \param[in] _world Pointer the World
  /// \param[in] _sdf Pointer the the SDF element of the plugin.
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief Callback function when collision occurs in the world.
  /// \param[in] _contacts List of all collisions from last simulation iteration
  private: void OnCollisionMsg(ConstContactsPtr &_contacts);

  /////////////////////////////////////////////////////////////////////////
  // Fields
 
  /// \brief Collision detection node subscriber
  private: transport::SubscriberPtr collisionSub;

  /// \brief gazebo node pointer
  private: transport::NodePtr collisionNode;

};

}  // namespace gazebo

#endif
