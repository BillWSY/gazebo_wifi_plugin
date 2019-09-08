#include "WifiReceiverPlugin.hh"

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"

using namespace gazebo;
using namespace sensors;
GZ_REGISTER_SENSOR_PLUGIN(WifiReceiverPlugin)

WifiReceiverPlugin::WifiReceiverPlugin() : SensorPlugin() {
}

WifiReceiverPlugin::~WifiReceiverPlugin() {
}

void WifiReceiverPlugin::Load(
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  // Get the parent sensor.
  this->parent_sensor_ =
      std::dynamic_pointer_cast<sensors::WirelessReceiver>(sensor);

  // Make sure the parent sensor is valid.
  if (!this->parent_sensor_) {
    gzerr << "WifiReceiverPlugin requires a Wireless Transmitter Sensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->update_connection_ = this->parent_sensor_->ConnectUpdated(
      std::bind(&WifiReceiverPlugin::UpdateImpl, this));

  // Make sure the parent sensor is active.
  this->parent_sensor_->SetActive(true);


  ros::NodeHandle node_handle;
  this->sensor_pub_ = node_handle.advertise<std_msgs::String>(
      this->parent_sensor_->Name() + "/essid", 1000);
}

bool WifiReceiverPlugin::UpdateImpl() {
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it) {
    if ((*it)->Type() == "wireless_transmitter") {
      sensors::WirelessTransmitterPtr transmit_sensor =
          std::dynamic_pointer_cast<sensors::WirelessTransmitter>(*it);

      std::string txEssid;

      double signal_strength = transmit_sensor->SignalStrength(
          this->parent_sensor_->Pose(), this->parent_sensor_->Gain());
      double tx_freq = transmit_sensor->Freq();
      std::string tx_essid = transmit_sensor->ESSID();

      std::cout << "Signal strengh: " << signal_strength << std::endl;
      std::cout << "TX frequency: " << tx_freq << std::endl;
      std::cout << "ESSID: " << tx_essid << std::endl;

      std_msgs::String msg;
      msg.data = tx_essid;
      sensor_pub_.publish(msg);

      // Discard if the frequency received is out of our frequency range,
      // or if the received signal strengh is lower than the sensivity
      if ((tx_freq < this->parent_sensor_->MinFreqFiltered())
          || (tx_freq > this->parent_sensor_->MaxFreqFiltered())
          || (signal_strength < this->parent_sensor_->Sensitivity())) {
        continue;
      }

      // TODO(shengye): Add this information to a ROS message and publish it.
    }
  }
  return true;
}
