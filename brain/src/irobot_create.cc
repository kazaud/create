#include "irobot_create.h"

#include <SerialStream.h>
#include <bitset>
#include <cassert>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include "tools/collection_utils.h"

using ::LibSerial::SerialStream;
using ::LibSerial::SerialStreamBuf;

//TODO: take into account the 15ms/sent command limit.

namespace create {
namespace {
// Push a 16 bits integer (high byte first) in a stream.
static void sendInt16(SerialStream* s, short i) {
  const unsigned char v1 = i & 0xFFFF;
  const unsigned char v2 = (i & 0xFFFF) >> 8;
  *s << v2 << v1;
}

///// Receive a 16 bits integer (high byte first) in a stream.
//static short receiveInt16(std::istream& s) {
//unsigned char v1;
//unsigned char v2;
//s >> v2 >> v1;
//return (v2 << 8) + v1;
//}
//
///// Peek that automatically reset the error flag if EOF is reached.
//static int safePeek(std::istream& s) {
//try {
//s.sync();
//if (s.rdbuf()->in_avail() == 0) {
//return EOF;
//}
//
//int tmp = s.peek();
//if (tmp == EOF) {
//s.clear();
//}
//return tmp;
//} catch (std::iostream::failure& e) {
//s.clear();
//}
//return EOF;
//}
//
///// Try to get a character (reset error flag on failure).
//static bool safeGet(std::istream& s, unsigned char& c) {
//try {
//s.sync();
//if (s.rdbuf()->in_avail() == 0) {
//return false;
//}
//unsigned char tmp = s.get();
//if (!s.good()) {
//s.clear();
//return false;
//}
//c = tmp;
//return true;
//} catch (std::iostream::failure& e) {
//s.clear();
//}
//return false;
//}
//
}//end of namespace

unique_ptr<Create> Create::MakeFromPort(const string& serial_port_name) {
  unique_ptr<SerialStream> stream = MakeUnique<SerialStream>(
      serial_port_name, SerialStreamBuf::BAUD_57600, SerialStreamBuf::CHAR_SIZE_8,
      SerialStreamBuf::PARITY_NONE, 1, SerialStreamBuf::FLOW_CONTROL_NONE);
  if (!stream->IsOpen()) {
    cout << "Unable to open stream to port " << serial_port_name << endl;
    return nullptr;
  }
  return unique_ptr < Create > (new Create(std::move(stream)));
}
Create::Create(unique_ptr<SerialStream> stream)
    : currentMode_(OpenInterface::MODE_OFF), stream_(std::move(stream)) {
  //Discard header and previous data.
  while (stream_->rdbuf()->in_avail() > 0) {
    stream_->ignore();
  }

  sendStartCommand();
}
//
Create::~Create() {
}

bool Create::sendBaudCommand(OpenInterface::Baud baud) {
  if (currentMode_ < OpenInterface::MODE_PASSIVE) {
    return false;
  }
  if (baud < 0 || baud > OpenInterface::BAUD_115200) {
    return false;
  }
  if (!ContainsKey(OpenInterface::kBaudMap, baud)) {
    return false;
  }

  const unsigned char op = OpenInterface::OPCODE_BAUD;
  const unsigned char b = baud;
  *stream_ << op << b;
  stream_->flush();

  stream_->SetBaudRate(OpenInterface::kBaudMap.at(baud));

  usleep(100 * 1000);
  return true;
}

bool Create::sendStartCommand() {
  if (currentMode_ == OpenInterface::MODE_PASSIVE) {
    return false;
  }
  const unsigned char op = OpenInterface::OPCODE_START;

  *stream_ << op;
  stream_->flush();
  currentMode_ = OpenInterface::MODE_PASSIVE;
  return true;
}

bool Create::sendSafeCommand() {
  if (currentMode_ == OpenInterface::MODE_SAFE) {
    return false;
  }
  const unsigned char op = OpenInterface::OPCODE_SAFE;
  *stream_ << op;
  stream_->flush();
  currentMode_ = OpenInterface::MODE_SAFE;
  return true;
}

bool Create::sendFullCommand() {
  if (currentMode_ == OpenInterface::MODE_FULL) {
    return false;
  }
  const unsigned char op = OpenInterface::OPCODE_FULL;
  *stream_ << op;
  stream_->flush();
  currentMode_ = OpenInterface::MODE_FULL;
  return true;
}

bool Create::sendDemoCommand(OpenInterface::Demo demo) {
  if (currentMode_ == OpenInterface::MODE_OFF) {
    return false;
  }
  if (demo != OpenInterface::DEMO_ABORT
      && (demo < 0 || demo > OpenInterface::DEMO_BANJO)) {
    return false;
  }
  const unsigned char op = OpenInterface::OPCODE_DEMO;
  const unsigned char d = demo;
  *stream_ << op << d;
  stream_->flush();
  currentMode_ = OpenInterface::MODE_PASSIVE;
  return true;
}

bool Create::sendDriveCommand(short velocity, short radius) {
  if (currentMode_ < OpenInterface::MODE_SAFE) {
    return false;
  }
  if (velocity < OpenInterface::VELOCITY_MIN || velocity > OpenInterface::VELOCITY_MAX
      || radius < OpenInterface::RADIUS_MIN || radius > OpenInterface::RADIUS_MAX) {
    return false;;
  }

  const unsigned char op = OpenInterface::OPCODE_DRIVE;

  *stream_ << op;
  sendInt16(stream_.get(), velocity);
  sendInt16(stream_.get(), radius);
  stream_->flush();
  return true;
}
//
//void Create::sendDriveCommand(short velocity, DriveCommand cmd)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//if (velocity < VELOCITY_MIN || velocity > VELOCITY_MAX) {
//throw InvalidArgument();
//}
//
//int arg = 0;
//switch (cmd) {
//case DRIVE_STRAIGHT:
//arg = 0x8000;
//break;
//case DRIVE_INPLACE_CLOCKWISE:
//arg = 0xFFFF;
//break;
//case DRIVE_INPLACE_COUNTERCLOCKWISE:
//arg = 0x0001;
//break;
//default:
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_DRIVE;
//stream_ << op;
//sendInt16(stream_, velocity);
//sendInt16(stream_, arg);
//stream_.flush();
//}
//
//void Create::sendDriveDirectCommand(short velocityL, short velocityR)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//if (velocityL < VELOCITY_MIN || velocityL > VELOCITY_MAX) {
//throw InvalidArgument();
//}
//if (velocityR < VELOCITY_MIN || velocityR > VELOCITY_MAX) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_DRIVE_DIRECT;
//stream_ << op;
//sendInt16(stream_, velocityR);
//sendInt16(stream_, velocityL);
//stream_.flush();
//}
//
//void Create::sendLedCommand(Led ledBits, unsigned char powerColor, unsigned char powerIntensity)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//if (ledBits < LED_NONE || ledBits > LED_ALL) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_LEDS;
//const unsigned char b = ledBits;
//stream_ << op << b << powerColor << powerIntensity;
//stream_.flush();
//}
//
//void Create::sendDigitalOutputsCommand(bool d0, bool d1, bool d2)
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_DIGITAL_OUTPUTS;
//const unsigned char v = d0 + d1 * 2 + d2 * 4;
//stream_ << op << v;
//stream_.flush();
//}
//
//void Create::sendPwmLowSideDriversCommand(unsigned char d1, unsigned char d2, unsigned char d3)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//if (d1 > LOW_SIDE_VELOCITY_MAX
//|| d2 > LOW_SIDE_VELOCITY_MAX
//|| d3 > LOW_SIDE_VELOCITY_MAX) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_PWM_LOW_SIDE_DRIVERS;
//stream_ << op << d1 << d2 << d3;
//stream_.flush();
//}
//
//void Create::sendLowSideDriversCommand(bool ld0, bool ld1, bool sd)
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_LOW_SIDE_DRIVERS;
//const unsigned char v = ld0 + ld1 * 2 + sd * 4;
//stream_ << op << v;
//stream_.flush();
//}
//
//void Create::sendIrCommand(unsigned char v)
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_SEND_IR;
//stream_ << op << v;
//stream_.flush();
//}
//
//void Create::sendSongCommand(unsigned char songN, const song_t& song)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//if (songN > SONG_MAX) {
//throw InvalidArgument();
//}
//if (song.size() > SONG_MAX_SIZE) {
//throw InvalidArgument();
//}
//
//std::stringstream ss;
//for (song_t::const_iterator it = song.begin();it != song.end();++it) {
//if (it->first < NO_NOTE || it->first > NOTE_MAX) {
//throw InvalidArgument();
//}
//ss << it->first << it->second;
//}
//const unsigned char op = OPCODE_SONG;
//const unsigned char size = song.size();
//stream_ << op << songN << size << ss.str();
//stream_.flush();
//}
//
////FIXME: check that song exist + song is not playing.
//void Create::sendPlaySongCommand(unsigned char v)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_SAFE) {
//throw CommandNotAvailable();
//}
//if (v > SONG_MAX) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_PLAY;
//stream_ << op << v;
//stream_.flush();
//}
//
////FIXME: check that the sensor can be queried successfully.
////Take into account the currents streamed sensors.
//void Create::sendSensorsCommand(SensorPacket packet)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//if (packet < SENSOR_GROUP_0
//|| packet > SENSOR_REQUESTED_LEFT_VELOCITY) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_SENSORS;
//const unsigned char p = packet;
//stream_ << op << p;
//
//queriedSensors_.push(packet);
//stream_.flush();
//}
//
///// Internal macro.
//#define MAKE_SENSOR_CMD(OP, CMD)
//  if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//    throw CommandNotAvailable();}
//  if (packets.size() > 255) {
//    throw InvalidArgument();}
//
//  std::stringstream ss;
//  for (sensorPackets_t::const_iterator it = packets.begin();
//       it != packets.end();++it)
//  {
//    if (*it < SENSOR_GROUP_0
//        || *it > SENSOR_REQUESTED_LEFT_VELOCITY) {
//      throw InvalidArgument();}
//    const unsigned char p = *it;
//    ss << p;
//    CMD;
//  }
//
//  const unsigned char op = (OP);
//  const unsigned char size = packets.size();
//  stream_ << op << size << ss.str();
//  stream_.flush()
//
////FIXME: check that the amount of data can be queried successfully.
////Take into account the currents streamed sensors.
//void Create::sendQueryListCommand(const sensorPackets_t& packets)
//throw(CommandNotAvailable, InvalidArgument) {
//MAKE_SENSOR_CMD(OPCODE_QUERY_LIST,
//queriedSensors_.push(*it));
//}
//
////FIXME: check that the amount of data can be streamed successfully.
//void Create::sendStreamCommand(const sensorPackets_t& packets)
//throw(CommandNotAvailable, InvalidArgument) {
//MAKE_SENSOR_CMD(OPCODE_STREAM,
//streamedSensors_.push_back(*it));
//}
//
//#undef MAKE_SENSOR_CMD
//
//void Create::sendPauseStreamCommand(StreamState state)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//if (state < 0 || state > STREAM_STATE_ON) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_PAUSE_RESUME_STREAM;
//const unsigned char st = state;
//stream_ << op << st;
//stream_.flush();
//}
//
//void Create::sendScriptCommand(const opcodes_t& opcodes)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//if (opcodes.size() > SCRIPT_MAX_SIZE) {
//throw InvalidArgument();
//}
//
//std::stringstream ss;
//for (opcodes_t::const_iterator it = opcodes.begin();
//it != opcodes.end();++it) {
//if (*it < OPCODE_START
//|| *it > OPCODE_WAIT_EVENT) {
//throw InvalidArgument();
//}
//const unsigned char opcode = *it;
//ss << opcode;
//}
//
//const unsigned char op = OPCODE_SCRIPT;
//const unsigned char size = opcodes.size();
//stream_ << op << size << ss.str();
//stream_.flush();
//}
//
//void Create::sendPlayScriptCommand()
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_PLAY_SCRIPT;
//stream_ << op;
//stream_.flush();
//}
//
//void Create::sendShowScriptCommand()
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_SHOW_SCRIPT;
//stream_ << op;
//stream_.flush();
//}
//
//void Create::sendWaitTimeCommand(unsigned char t)
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_WAIT_TIME;
//stream_ << op << t;
//stream_.flush();
//}
//
//void Create::sendWaitDistanceCommand(short d)
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_WAIT_DISTANCE;
//stream_ << op;
//sendInt16(stream_, d);
//stream_.flush();
//}
//
//void Create::sendWaitAngleCommand(short a)
//throw(CommandNotAvailable) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//
//const unsigned char op = OPCODE_WAIT_ANGLE;
//stream_ << op;
//sendInt16(stream_, a);
//stream_.flush();
//}
//
//void Create::sendWaitEventCommand(Event event, EventState state)
//throw(CommandNotAvailable, InvalidArgument) {
//if (currentMode_ < IROBOT_CREATE_PASSIVE) {
//throw CommandNotAvailable();
//}
//if (event < EVENT_WHEEL_DROP || event > EVENT_OI_MODE_PASSIVE) {
//throw InvalidArgument();
//}
//if (state < EVENT_OCCURRING || state > EVENT_NOT_OCCURRING) {
//throw InvalidArgument();
//}
//
//const unsigned char op = OPCODE_WAIT_EVENT;
//const char e = (state == EVENT_OCCURRING)?event:-event;
//stream_ << op << e;
//stream_.flush();
//}
//
//void Create::updateSensors() {
//while (!!stream_.good() && !queriedSensors_.empty()) {
//if (!!stream_.good() && safePeek(stream_) == STREAM_HEADER) {
//readStream();
//}
//if (!readSensorPacket()) {
//if (stream_.rdbuf()->in_avail() > 0) {
//stream_.ignore();
//} else {
//break;
//}
//}
//}
//
//while (!!stream_.good() && stream_.rdbuf()->in_avail() > 0) {
//if (safePeek(stream_) == STREAM_HEADER) {
//readStream();
//} else {
//stream_.ignore();
//}
//}
//}
//
//bool Create::readSensorPacket() {
//if (!!queriedSensors_.empty()) {
//return false;
//}
//if (!!readSensorPacket(queriedSensors_.front(), stream_)) {
//queriedSensors_.pop();
//return true;
//}
//return false;
//}
//
//
//#define GEN_SENSOR(PACKET, SENSOR, TYPE)
//case PACKET:
//{
//  unsigned char v = 0;
//  if (!safeGet(stream, v)) {
//    return false;}
//  SENSOR = static_cast<TYPE>(v);
//}
//break
//
//#define CHAR_SENSOR(PACKET, SENSOR)
//  GEN_SENSOR(PACKET, SENSOR, char)
//
//#define UCHAR_SENSOR(PACKET, SENSOR)
//  GEN_SENSOR(PACKET, SENSOR, unsigned char)
//
//#define INT_SENSOR(PACKET, SENSOR)
//case PACKET:
//{
//  SENSOR = receiveInt16(stream);
//}
//break
//
//#define GROUP_SENSOR(PACKET, MIN, MAX)
//  {
//  case PACKET:
//    bool res = true;
//    for (SensorPacket sensor = static_cast<SensorPacket>(MIN);
//         sensor < static_cast<SensorPacket>(MAX);
//         sensor = static_cast<SensorPacket>(sensor + 1)) {
//      res = res && readSensorPacket(sensor, stream);}
//    if (!res) {
//      return false;}
//  }
//  break
//
//bool Create::readSensorPacket(SensorPacket sensor, std::istream& stream) {
//if (!stream.good()) {
//return false;
//}
//if (safePeek(stream) == STREAM_HEADER) {
//return false;
//}
//
//switch (sensor) {
//GROUP_SENSOR(SENSOR_GROUP_0, 7, 26);
//GROUP_SENSOR(SENSOR_GROUP_1, 7, 16);
//GROUP_SENSOR(SENSOR_GROUP_2, 17, 20);
//GROUP_SENSOR(SENSOR_GROUP_3, 21, 26);
//GROUP_SENSOR(SENSOR_GROUP_4, 27, 34);
//GROUP_SENSOR(SENSOR_GROUP_5, 35, 42);
//GROUP_SENSOR(SENSOR_GROUP_6, 7, 42);
//
//case SENSOR_BUMPS_WHEELS_DROPS:
//{
//std::bitset<8> v = 0;
//unsigned char tmp = 0;
//if (!safeGet(stream, tmp)) {
//return false;
//}
//v = tmp;
//wheeldropCaster_ = v.test(SENSOR_BIT_WHEELDROP_CASTER);
//wheeldropLeft_ = v.test(SENSOR_BIT_WHEELDROP_LEFT);
//wheeldropRight_ = v.test(SENSOR_BIT_WHEELDROP_RIGHT);
//bumpLeft_ = v.test(SENSOR_BIT_BUMP_LEFT);
//bumpRight_ = v.test(SENSOR_BIT_BUMP_RIGHT);
//break;
//}
//
//UCHAR_SENSOR(SENSOR_WALL, wall_);
//UCHAR_SENSOR(SENSOR_CLIFF_LEFT, cliffLeft_);
//UCHAR_SENSOR(SENSOR_CLIFF_FRONT_LEFT, cliffFrontLeft_);
//UCHAR_SENSOR(SENSOR_CLIFF_FRONT_RIGHT, cliffFrontRight_);
//UCHAR_SENSOR(SENSOR_CLIFF_RIGHT, cliffRight_);
//UCHAR_SENSOR(SENSOR_VIRTUAL_WALL, virtualWall_);
//
//case SENSOR_OVERCURRENTS:
//{
//std::bitset<8> v = 0;
//unsigned char tmp = 0;
//if (!safeGet(stream, tmp)) {
//return false;
//}
//v = tmp;
//leftWheelOvercurrent_ = v.test(SENSOR_BIT_LEFTWHEELOVERCURRENT);
//rightWheelOvercurrent_ = v.test(SENSOR_BIT_RIGHTWHEELOVERCURRENT);
//ld2Overcurrent_ = v.test(SENSOR_BIT_LD2OVERCURRENT);
//ld1Overcurrent_ = v.test(SENSOR_BIT_LD1OVERCURRENT);
//ld0Overcurrent_ = v.test(SENSOR_BIT_LD0OVERCURRENT);
//break;
//}
//
////Unused.
//case 15:
//case 16:
//{
//unsigned char tmp = 0;
//if (!safeGet(stream, tmp)) {
//return false;
//}
//break;
//}
//
//UCHAR_SENSOR(SENSOR_IR, ir_);
//
//case SENSOR_BUTTONS:
//{
//std::bitset<8> v = 0;
//unsigned char tmp = 0;
//if (!safeGet(stream, tmp)) {
//return false;
//}
//v = tmp;
//advanceButton_ = v.test(SENSOR_BIT_ADVANCEBUTTON);
//playButton_ = v.test(SENSOR_BIT_PLAYBUTTON);
//break;
//}
//
//INT_SENSOR(SENSOR_DISTANCE, distance_);
//INT_SENSOR(SENSOR_ANGLE, angle_);
//
//GEN_SENSOR(SENSOR_CHARGING_STATE, chargingState_, ChargingState);
//
//INT_SENSOR(SENSOR_VOLTAGE, batteryVoltage_);
//INT_SENSOR(SENSOR_CURRENT, batteryCurrent_);
//CHAR_SENSOR(SENSOR_BATTERY_TEMPERATURE, batteryTemperature_);
//INT_SENSOR(SENSOR_BATTERY_CHARGE, batteryCharge_);
//INT_SENSOR(SENSOR_BATTERY_CAPACITY, batteryCapacity_);
//INT_SENSOR(SENSOR_WALL_SIGNAL, wallSignal_);
//INT_SENSOR(SENSOR_CLIFF_LEFT_SIGNAL, cliffLeftSignal_);
//INT_SENSOR(SENSOR_CLIFF_FRONT_LEFT_SIGNAL, cliffFrontLeftSignal_);
//INT_SENSOR(SENSOR_CLIFF_FRONT_RIGHT_SIGNAL, cliffFrontRightSignal_);
//INT_SENSOR(SENSOR_CLIFF_RIGHT_SIGNAL, cliffRightSignal_);
//
//case SENSOR_CARGO_BAY_DIGITAL_INPUT:
//{
//std::bitset<8> v = 0;
//unsigned char tmp = 0;
//if (!safeGet(stream, tmp)) {
//return false;
//}
//v = tmp;
//deviceDetect_ = v.test(SENSOR_BIT_DEVICEDETECT);
//digitalInput3_ = v.test(SENSOR_BIT_DIGITALINPUT3);
//digitalInput2_ = v.test(SENSOR_BIT_DIGITALINPUT2);
//digitalInput1_ = v.test(SENSOR_BIT_DIGITALINPUT1);
//digitalInput0_ = v.test(SENSOR_BIT_DIGITALINPUT0);
//break;
//}
//
//INT_SENSOR(SENSOR_CARGO_BAY_ANALOG_SIGNAL, analogSignal_);
//
//case SENSOR_CHARGING_SOURCES_AVAILABLE:
//{
//std::bitset<8> v = 0;
//unsigned char tmp = 0;
//if (!safeGet(stream, tmp)) {
//return false;
//}
//v = tmp;
//homeBaseChargerAvailable_ =
//v.test(SENSOR_BIT_HOMEBASECHARGERAVAILABLE);
//internalChargerAvailable_ =
//v.test(SENSOR_BIT_INTERNALCHARGERAVAILABLE);
//break;
//}
//
////FIXME: trigger a warning if not corresponding to current value?
//GEN_SENSOR(SENSOR_OI_MODE, currentMode_, Mode);
//
//UCHAR_SENSOR(SENSOR_SONG_NUMBER, songNumber_);
//UCHAR_SENSOR(SENSOR_SONG_PLAYING, songPlaying_);
//UCHAR_SENSOR(SENSOR_NUMBER_STREAM_PACKETS, streamPackets_);
//
//INT_SENSOR(SENSOR_REQUESTED_VELOCITY, requestedVelocity_);
//INT_SENSOR(SENSOR_REQUESTED_RADIUS, requestedRadius_);
//INT_SENSOR(SENSOR_REQUESTED_LEFT_VELOCITY, requestedLeftVelocity_);
//INT_SENSOR(SENSOR_REQUESTED_RIGHT_VELOCITY, requestedRightVelocity_);
//
//default:
//break;
//}
//return true;
//}
//
////FIXME: throw exceptions.
//void Create::readStream() {
//if (!stream_.good()) {
//return;
//}
//if (safePeek(stream_) != STREAM_HEADER) {
//return;
//}
//
////Ignore header.
//stream_.ignore();
//
//std::stringstream ss;
//unsigned char checksum = STREAM_HEADER;
//unsigned char size = 0;
//
//stream_ >> size;
//checksum += size;
//for (unsigned i = 0;i < size;++i) {
//unsigned char c;
//stream_ >> c;
//ss << c;
//checksum += c;
//}
//
//unsigned char sentChecksum = 0;
//stream_ >> sentChecksum;
//
//if (checksum + sentChecksum != 256) {
//return;
//}
//
//for (sensorPackets_t::const_iterator it = streamedSensors_.begin();
//it != streamedSensors_.end();++it) {
//unsigned char packet = 0;
//ss >> packet;
//if (packet != *it) {
//return;
//}
//if (!readSensorPacket(*it, ss)) {
//return;
//}
//}
//}
//
//#define MK_SENSOR_GETTER(TYPE, NAME)
//  TYPE
//  Create::NAME()
//  {
//    updateSensors();
//    return NAME ## _;
//  }
//
//MK_SENSOR_GETTER(bool, wheeldropCaster)
//MK_SENSOR_GETTER(bool, wheeldropLeft)
//MK_SENSOR_GETTER(bool, wheeldropRight)
//MK_SENSOR_GETTER(bool, bumpLeft)
//MK_SENSOR_GETTER(bool, bumpRight)
//MK_SENSOR_GETTER(bool, wall)
//MK_SENSOR_GETTER(bool, cliffLeft)
//MK_SENSOR_GETTER(bool, cliffFrontLeft)
//MK_SENSOR_GETTER(bool, cliffFrontRight)
//MK_SENSOR_GETTER(bool, cliffRight)
//MK_SENSOR_GETTER(bool, deviceDetect)
//MK_SENSOR_GETTER(bool, digitalInput3)
//MK_SENSOR_GETTER(bool, digitalInput2)
//MK_SENSOR_GETTER(bool, digitalInput1)
//MK_SENSOR_GETTER(bool, digitalInput0)
//MK_SENSOR_GETTER(short, analogSignal)
//MK_SENSOR_GETTER(bool, homeBaseChargerAvailable)
//MK_SENSOR_GETTER(bool, internalChargerAvailable)
//MK_SENSOR_GETTER(bool, virtualWall)
//MK_SENSOR_GETTER(bool, leftWheelOvercurrent)
//MK_SENSOR_GETTER(bool, rightWheelOvercurrent)
//MK_SENSOR_GETTER(bool, ld2Overcurrent)
//MK_SENSOR_GETTER(bool, ld1Overcurrent)
//MK_SENSOR_GETTER(bool, ld0Overcurrent)
//MK_SENSOR_GETTER(unsigned char, ir)
//MK_SENSOR_GETTER(bool, advanceButton)
//MK_SENSOR_GETTER(bool, playButton)
//MK_SENSOR_GETTER(short, distance)
//MK_SENSOR_GETTER(short, angle)
//MK_SENSOR_GETTER(Create::ChargingState, chargingState)
//MK_SENSOR_GETTER(short, batteryVoltage)
//MK_SENSOR_GETTER(short, batteryCurrent)
//MK_SENSOR_GETTER(short, batteryTemperature)
//MK_SENSOR_GETTER(short, batteryCharge)
//MK_SENSOR_GETTER(short, batteryCapacity)
//MK_SENSOR_GETTER(short, wallSignal)
//MK_SENSOR_GETTER(short, cliffLeftSignal)
//MK_SENSOR_GETTER(short, cliffFrontLeftSignal)
//MK_SENSOR_GETTER(short, cliffFrontRightSignal)
//MK_SENSOR_GETTER(short, cliffRightSignal)
//MK_SENSOR_GETTER(unsigned char, songNumber)
//MK_SENSOR_GETTER(bool, songPlaying)
//MK_SENSOR_GETTER(unsigned char, streamPackets)
//MK_SENSOR_GETTER(short, requestedVelocity)
//MK_SENSOR_GETTER(short, requestedRadius)
//MK_SENSOR_GETTER(short, requestedLeftVelocity)
//MK_SENSOR_GETTER(short, requestedRightVelocity)

OpenInterface::Mode Create::mode() {
  return currentMode_;
}
}  //end of namespace create
