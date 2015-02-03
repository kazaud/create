/**
 * \file irobot-create.h
 *
 * \brief Declaration of the Create class.
 */

#ifndef IROBOT_CREATE_H
#define IROBOT_CREATE_H

#include "irobot_create_open_interface.h"
#include "tools/make_unique.h"
#include <SerialStream.h>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace create {
//Class for communicating with iRobot hardware using the OpenInterface.
class Create {
public:
  //brief Construct an instance of Create.
  //param serial_port_name name of the serial port to use.
  static unique_ptr<Create> MakeFromPort (const string& serial_port_name);
  virtual ~Create();

private:
  //Construct an instance of Create using serial port communication.
  //stream Stream used for communication. Will check fail if stream is not open and valid
  explicit Create(unique_ptr<LibSerial::SerialStream> stream);

  bool sendBaudCommand (OpenInterface::Baud speed);
  //Automatically done when the class is instanciated.
  void sendStartCommand ();
  void sendSafeCommand ();
  void sendFullCommand ();
  void sendDemoCommand (OpenInterface::Demo demo);

  //Drive the robot.
  //param v Robot's velocity.
  //param r Robot's turning radius.
  //see sendDriveDirectCommand
  bool sendDriveCommand (short v, short r);

  //\Drive the robot using a special driving mode.
  //v Robot's velocity.
  //command Special driving mode.
  bool sendDriveCommand (short v, OpenInterface::DriveCommand dc);

  //Drive the two wheels separately.
  //vr Right wheel's velocity.
  //vl Left wheel's velocity.
  bool sendDriveDirectCommand (short vr, short vl);

  //Change led status.
  //l Switch on/off advance or play leds.
  //c Power led color.
  //i Power led light intensity.
  bool sendLedCommand (OpenInterface::Led l, unsigned char c, unsigned char i);

  //Change the digital output values.
  //d1 Switch on/off first digital output.
  //d2 Switch on/off second digital output.
  //d3 Switch on/off third digital output.
  void sendDigitalOutputsCommand (bool d1, bool d2, bool d3);

  //Drive low side drivers with variable power.
  //lsd1 Low side driver 1 power.
  //lsd2 Low side driver 2 power.
  //lsd3 Low side driver 3 power.
  bool sendPwmLowSideDriversCommand (unsigned char lsd1, unsigned char lsd2, unsigned char lsd3);

  //Switch on/off low side drivers.
  //lsd1 Switch on/off low side driver 1.
  //lsd2 Switch on/off low side driver 2.
  //lsd3 Switch on/off low side driver 3.
  //\see sendPwmLowSideDriversCommand
  void sendLowSideDriversCommand (bool lsd1, bool lsd2, bool lsd3);

  //Send an IR command on low side driver 1.
  //The data will be encoded using the iRobot's Create receiver format.
  //v Sent value.
  void sendIrCommand (unsigned char v);

  //Define a song.
  //sid Song id.
  //song Song to send.
  bool sendSongCommand (unsigned char sid, const vector<OpenInterface::Note>& song);

  //Play a song
  //sid Sond id to play.
  bool sendPlaySongCommand (unsigned char sid);

  //Request the robot to send a particular sensor packet.
  //sp Sensor packet to send.
  bool sendSensorsCommand (OpenInterface::SensorPacket sp);

  //Request the robot to send a list of sensor packets.
  //lsp List of sensor packets to send
  bool sendQueryListCommand (const vector<OpenInterface::SensorPacket>& lsp);

  //Request the robot to start streaming some sensor packets.
  //After the streaming is started, the requested values will be sent
  //every 15ms.
  //lsp List of sensor packets to stream.
  bool sendStreamCommand (const vector<OpenInterface::SensorPacket>& lsp);

  //Pause or resume sensor streaming.
  //st Stream state to set.
  bool sendPauseStreamCommand (OpenInterface::StreamState st);

  //Send a script.
  //script Script to send.
  bool sendScriptCommand (const vector<OpenInterface::Opcode>& script);

  //Play the current stored script.
  void sendPlayScriptCommand ();

  //Show the stored script.
  void sendShowScriptCommand ();

  //Make the robot wait for a specific amount of time.
  //t Time to wait in tenth of a second (resolution of 15 ms).
  void sendWaitTimeCommand (unsigned char t);

  //Make the robot wait until it travelled a certain distance.
  //d Distance to travel
  void sendWaitDistanceCommand (short d);

  //Make the robot wait until it has rotated enough.
  //a Angle (in degrees)
  void sendWaitAngleCommand (short a);

  //Make the robot wait for a specific event.
  //e Wait for this event.
  //es Wait for this event state.
  bool sendWaitEventCommand (OpenInterface::Event e, OpenInterface::EventState es = OpenInterface::EVENT_OCCURRING);

//// Read the stream to update sensors values.
//// This handles both streamed data and specific queries
//// through querylist or sensors commands.
//void updateSensors ();
//
//// Read streamed sensors and update sensors values.
//void readStream ();
//
//// \}
//
//// \{
//
  OpenInterface::Mode mode ();

  bool wheeldropCaster() const {return wheeldropCaster_;}
  bool wheeldropLeft() const {return wheeldropLeft_;}
  bool wheeldropRight() const {return wheeldropRight_;}
  bool bumpLeft() const {return bumpLeft_;}
  bool bumpRight() const {return bumpRight_;}

  //Get wall sensor value.
  bool wall() const {return wall_;}
  bool cliffLeft() const {return cliffLeft_;}
  bool cliffFrontLeft() const {return cliffFrontLeft_;}
  bool cliffFrontRight() const {return cliffFrontRight_;}
  bool cliffRight() const {return cliffRight_;}
  bool deviceDetect() const {return deviceDetect_;}
  bool digitalInput3() const {return digitalInput3_;}
  bool digitalInput2() const {return digitalInput2_;}
  bool digitalInput1() const {return digitalInput1_;}
  bool digitalInput0() const {return digitalInput0_;}
  short analogSignal() const {return analogSignal_;}
  bool homeBaseChargerAvailable() const {return homeBaseChargerAvailable_;}
  bool internalChargerAvailable() const {return internalChargerAvailable_;}
  bool virtualWall() const {return virtualWall_;}
  bool leftWheelOvercurrent() const {return leftWheelOvercurrent_;}
  bool rightWheelOvercurrent() const {return rightWheelOvercurrent_;}
  bool ld2Overcurrent() const {return ld2Overcurrent_;}
  bool ld1Overcurrent() const {return ld1Overcurrent_;}
  bool ld0Overcurrent() const {return ld0Overcurrent_;}
  unsigned char ir() const {return ir_;}
  bool advanceButton() const {return advanceButton_;}
  bool playButton() const {return playButton_;}
  short distance() const {return distance_;}
  short angle() const {return angle_;}
  OpenInterface::ChargingState chargingState() const {return chargingState_;}
  short batteryVoltage() const {return batteryVoltage_;}
  short batteryCurrent() const {return batteryCurrent_;}
  short batteryTemperature() const {return batteryTemperature_;}
  short batteryCharge() const {return batteryCharge_;}
  short batteryCapacity() const {return batteryCapacity_;}
  short wallSignal() const {return wallSignal_;}
  short cliffLeftSignal() const {return cliffLeftSignal_;}
  short cliffFrontLeftSignal() const {return cliffFrontLeftSignal_;}
  short cliffFrontRightSignal() const {return cliffFrontRightSignal_;}
  short cliffRightSignal() const {return cliffRightSignal_;}
  unsigned char songNumber() const {return songNumber_;}
  bool songPlaying() const {return songPlaying_;}
  unsigned char streamPackets() const {return streamPackets_;}
  short requestedVelocity() const {return requestedVelocity_;}
  short requestedRadius() const {return requestedRadius_;}
  short requestedLeftVelocity() const {return requestedLeftVelocity_;}
  short requestedRightVelocity() const {return requestedRightVelocity_;}

protected:
//// Read a specific sensor packet and update sensors values.
//// The next expected sensor packet will be read.
//bool readSensorPacket ();
//
//// Read sensor packet on a specified stream, update sensor values.
//bool readSensorPacket(SensorPacket, std::istream &);
//
//// Code shared between constructors.
//void init () throw(InvalidArgument);
//
//Current robot's mode.
  OpenInterface::Mode currentMode_;

  //Stream used for communication.
  unique_ptr<LibSerial::SerialStream> stream_;

  //List of sensors currently streamed.
  vector<OpenInterface::SensorPacket> streamedSensors_;

  //List of queried packets
  //Sensor packets that have to be read but which are not streamed.
  queue<OpenInterface::SensorPacket> queriedSensors_;

  //Wheeldrop caster
  bool wheeldropCaster_;
  //Wheeldrop left
  bool wheeldropLeft_;
  //Wheeldrop right
  bool wheeldropRight_;
  //Bump left
  bool bumpLeft_;
  //Bump right
  bool bumpRight_;
  //Wall sensor.
  bool wall_;
  //Left cliff sensor.
  bool cliffLeft_;
  //Front left cliff sensor.
  bool cliffFrontLeft_;
  //Front right cliff sensor.
  bool cliffFrontRight_;
  //Right cliff sensor.
  bool cliffRight_;
  //Device detect pin state.
  bool deviceDetect_;
  //Digital input 3 pin state.
  bool digitalInput3_;
  //Digital input 2 pin state.
  bool digitalInput2_;
  //Digital input 1 pin state.
  bool digitalInput1_;
  //Digital input 0 pin state.
  bool digitalInput0_;
  //Analog signal pin state.
  short analogSignal_;
  //Is the home base charger available?
  bool homeBaseChargerAvailable_;
  //Is the internal charger available?
  bool internalChargerAvailable_;
  //Virtual wall sensor.
  bool virtualWall_;
  //Left wheel overcurrent (true = overcurrent).
  bool leftWheelOvercurrent_;
  //Right wheel overcurrent (true = overcurrent).
  bool rightWheelOvercurrent_;
  //LD2 overcurrent (true = overcurrent).
  bool ld2Overcurrent_;
  //LD1 overcurrent (true = overcurrent).
  bool ld1Overcurrent_;
  //LD0 overcurrent (true = overcurrent).
  bool ld0Overcurrent_;
  //Ir sensor.
  unsigned char ir_;
  //Advance button state (true = pushed).
  bool advanceButton_;
  //Play button (true = pushed).
  bool playButton_;
  //Distance sensor.
  short distance_;
  //Angle sensor.
  short angle_;
  //Current charging state.
  OpenInterface::ChargingState chargingState_;
  //Battery voltage.
  short batteryVoltage_;
  //Battery current.
  short batteryCurrent_;
  //Battery temperature.
  short batteryTemperature_;
  //Battery charge.
  short batteryCharge_;
  //Battery capacity.
  short batteryCapacity_;
  //Distance from wall.
  short wallSignal_;
  //Distance from cliff (left).
  short cliffLeftSignal_;
  //Distance from cliff (front left).
  short cliffFrontLeftSignal_;
  //Distance from cliff (front right).
  short cliffFrontRightSignal_;
  //Distance from cliff (right).
  short cliffRightSignal_;
  //Current song number.
  unsigned char songNumber_;
  //Is a song currently played?
  bool songPlaying_;
  //Number of streamed packets.
  unsigned char streamPackets_;
  //Last requested velocity (drive command).
  short requestedVelocity_;
  //Last requested radius (drive command).
  short requestedRadius_;
  //Last requested left velocity (drive direct command).
  short requestedLeftVelocity_;
  //Last requested right velocity (drive direct command).
  short requestedRightVelocity_;
};
} //end of namespace create

#endif //! IROBOT_CREATE_H
