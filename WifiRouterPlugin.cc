#include "WifiRouterPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(WifiRouterPlugin)

WifiRouterPlugin::WifiRouterPlugin() : SensorPlugin()
{
}

WifiRouterPlugin::~WifiRouterPlugin()
{
}

void WifiRouterPlugin::Load(
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  // Get the parent sensor.
  this->parent_sensor_ =
      std::dynamic_pointer_cast<sensors::WirelessTransmitter>(sensor);

  // Make sure the parent sensor is valid.
  if (!this->parent_sensor_) {
    gzerr << "WifiRouterPlugin requires a Wireless Transmitter Sensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->update_connection_ = this->parent_sensor_->ConnectUpdated(
      std::bind(&WifiRouterPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parent_sensor_->SetActive(true);

  std::string essid;
  essid = this->parent_sensor_->ESSID ();

  std::cout << " ESSID:" << essid << "\n";
}

void WifiRouterPlugin::OnUpdate() {
  std::string essid;
  essid = this->parent_sensor_->ESSID ();
}
