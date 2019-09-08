#ifndef _GAZEBO_WIFI_ROUTER_PLUGIN_HH_
#define _GAZEBO_WIFI_ROUTER_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo {

class WifiRouterPlugin : public SensorPlugin
{
  /// \brief Constructor.
  public: WifiRouterPlugin();

  /// \brief Destructor.
  public: virtual ~WifiRouterPlugin();

  /// \brief Load the sensor plugin.
  /// \param[in] sensor Pointer to the sensor that loaded this plugin.
  /// \param[in] sdf SDF element that describes the plugin.
  public: virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  private: virtual void OnUpdate();

  private: sensors::WirelessTransmitterPtr parent_sensor_;

  private: event::ConnectionPtr update_connection_;
};

}  // namespace gazebo
#endif
