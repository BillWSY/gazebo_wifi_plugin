#include "WifiReceiverPlugin.hh"

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
}

bool WifiReceiverPlugin::UpdateImpl() {
  std::string txEssid;
  // msgs::WirelessNodes msg;
  double rxPower;
  double txFreq;

  sensors::SensorPtr sensor_ptr;
  sensor_ptr = SensorManager::Instance()->GetSensor("wirelessTransmitter");

  sensors::WirelessTransmitterPtr transmit_sensor;
  transmit_sensor = std::dynamic_pointer_cast<sensors::WirelessTransmitter>(
      sensor_ptr);

  std::cout << "Connected to: " + transmit_sensor->ESSID() + "\n";
  double signal_strength;
  signal_strength = transmit_sensor->SignalStrength(
      this->parent_sensor_->Pose(), this->parent_sensor_->Gain());
  std::cout << "Signal strengh: " << signal_strength << "\n";

  ignition::math::Pose3d myPos = this->parent_sensor_->Pose();
  std::cout << "Pose: " << myPos << "\n";

  /*for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
  {
    if ((*it)->GetType() == "wireless_transmitter")
    {


      sensors::WirelessTransmitterPtr parentSensor;
      this->parent_sensor_ = (*it)->GetSensor("wireless_transmitter");
      boost::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
           boost::static_pointer_cast<WirelessTransmitter>(*it);

      txFreq = transmitter->GetFreq();
      rxPower = transmitter->GetSignalStrength(myPos, this->GetGain());

      // Discard if the frequency received is out of our frequency range,
      // or if the received signal strengh is lower than the sensivity
      if ((txFreq < this->GetMinFreqFiltered()) ||
           (txFreq > this->GetMaxFreqFiltered()) ||
           (rxPower < this->GetSensitivity()))
      {
        continue;
      }

      txEssid = transmitter->GetESSID();

      msgs::WirelessNode *wirelessNode = msg.add_node();
      wirelessNode->set_essid(txEssid);
      wirelessNode->set_frequency(txFreq);
      std::cout << txEssid << "\n";
    }
  }*/


  return true;
}
