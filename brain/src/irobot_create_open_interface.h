/*
 * enums.h
 *
 *  Created on: Jan 31, 2015
 *      Author: jkobe
 */

#ifndef ENUMS_H_
#define ENUMS_H_

#include <vector>
#include <initializer_list>
#include <SerialStream.h>
#include "tools/make_unique.h"
#include "tools/using_std_stuff.h"

namespace create {
class OpenInterface {
public:
  struct Note {
    unsigned char frequency;
    unsigned char duration;
  };
  typedef vector<Note> Song;
  //iRobot Create modes
  //The mode defines which commands are available and
  //how the robot behaves when some events happens
  //like wheel drop or if a cliff is detected.
  enum Mode {
    //Start-up mode, no command can be sent except start.
    //Create::sendStartCommand
    MODE_OFF,
    //Passive mode, actuators can not be controlled.
    MODE_PASSIVE,
    //Safe mode, all commands are available.
    //iRobot Create will stop if the wheels are dropped,
    //if a cliff is detected or if the charger is plugged in
    //and powered.
    MODE_SAFE,
    //Full mode, all commands are available.
    //No safety check is done in this mode. The robot
    //may fall from cliffs and will keep running if the
    //wheels are dropped.
    MODE_FULL
  };

  /// Enumerate all possible communication speeds.
  enum Baud {
    //Communicate at 300 bauds per second.
    BAUD_300 = 0,
    //Communicate at 600 bauds per second.
    BAUD_600 = 1,
    //Communicate at 1200 bauds per second.
    BAUD_1200 = 2,
    //Communicate at 2400 bauds per second.
    BAUD_2400 = 3,
    //Communicate at 4800 bauds per second.
    BAUD_4800 = 4,
    //Communicate at 9600 bauds per second.
    BAUD_9600 = 5,
    //Communicate at 14400 bauds per second.
    BAUD_14400 = 6,
    //Communicate at 19200 bauds per second.
    BAUD_19200 = 7,
    //Communicate at 28800 bauds per second.
    BAUD_28800 = 8,
    //Communicate at 38400 bauds per second.
    BAUD_38400 = 9,
    //Communicate at 57600 bauds per second.
    BAUD_57600 = 10,
    //Communicate at 115200 bauds per second.
    //In this mode, be sure to keep at least 15 µs between each command.
    BAUD_115200 = 11
  };

  static const map<Baud, LibSerial::SerialStreamBuf::BaudRateEnum> kBaudMap;

  //Enumerate built-in demos.
  enum Demo {
    //iRobot Create covers an entire room using a combination of
    //behaviors,such as bouncing off of walls, following walls, and
    //spiraling.
    DEMO_COVER = 0,

    //Identical to the Cover demo, with one exception; if iRobot Create
    //sees the Home Base’s* infrared signals, it uses these to move
    //towards the Home Base and dock with it.
    DEMO_COVER_AND_DOCK = 1,

    //iRobot Create spirals outward, then inward, to cover an area around
    //its starting position.
    DEMO_SPOT_COVER = 2,

    //iRobot Create tries to follow around the edges of a room using its
    //wall sensor and bumper.
    DEMO_MOUSE = 3,

    //iRobot Create continuously drives in a figure 8 pattern.
    DEMO_DRIVE_FIGURE_EIGHT = 4,

    //iRobot Create drives forward when pushed from behind. If iRobot
    //Create hits an obstacle while driving, it drives away from the
    //obstacle.
    DEMO_WIMP = 5,

    //iRobot Create drives toward a Virtual Wall when the back and sides
    //of its Omnidirectional IR Receiver are covered with black
    //electrical tape. When it touches the Virtual Wall or another
    //obstacle, it stops.
    DEMO_HOME = 6,

    //Identical to the Home demo, except iRobot Create goes back and
    //forth between multiple Virtual Walls by bumping into one, turning
    //around, driving to the next Virtual Wall, bumping into it and
    //turning around to bump into the next Virtual Wall.
    DEMO_TAG = 7,

    //iRobot Create plays the notes of Pachelbel’s Canon in sequence when
    //its cliff sensors are activated.
    DEMO_PACHELBEL = 8,

    //iRobot Create’s four cliff sensors play the notes of a chord,
    //depending on how the bumper is pressed:
    //* No bumper: G major
    //* Right or left bumper: D major7
    //* Both bumpers (center): C major
    DEMO_BANJO = 9,

    //Abort current running demo.
    DEMO_ABORT = 255
  };

  //Enumerates special driving commands.
  enum DriveCommand {
    //Drive straight.
    DRIVE_STRAIGHT,
    //Turn in place clockwise.
    DRIVE_INPLACE_CLOCKWISE,
    //Turn in place counter-clockwise.
    DRIVE_INPLACE_COUNTERCLOCKWISE
  };

  //Enumerate available leds (and combinations).
  //The power led is not in this list because it is handled
  //differently (as its color is not fixed).
  enum Led {
    //No led.
    LED_NONE = 0,
    //Play led only.
    LED_PLAY = 2,
    //Advance led only.
    LED_ADVANCE = 8,
    //All leds.
    LED_ALL = LED_PLAY | LED_ADVANCE
  };

  //Activate or disable sensor streaming.
  enum StreamState {
    //Disable sensor streaming.
    STREAM_STATE_OFF = 0,
    //Activate sensor streaming.
    STREAM_STATE_ON = 1
  };

  //Enumerate possible charging states.
  enum ChargingState {
    CHARGING_STATE_NOT_CHARGING = 0,
    CHARGING_STATE_RECONDITIONING_CHARGING = 1,
    CHARGING_STATE_FULL_CHARGING = 2,
    CHARGING_STATE_TRICKLE_CHARGING = 3,
    CHARGING_STATE_WAITING = 4,
    CHARGING_STATE_CHARGING_FAULT_CONDITION = 5
  };

  //Enumerate available events.
  enum Event {
    //A wheel is dropped.
    EVENT_WHEEL_DROP = 1,
    //Front wheel is dropped.
    EVENT_FRONT_WHEEL_DROP = 2,
    //Left wheel is dropped.
    EVENT_LEFT_WHEEL_DROP = 3,
    //Right wheel is dropped.
    EVENT_RIGHT_WHEEL_DROP = 4,
    //Front part has bumped.
    EVENT_BUMP = 5,
    //Left part has bumped.
    EVENT_LEFT_BUMP = 6,
    //Right part has bumped.
    EVENT_RIGHT_BUMP = 7,
    //A virtual wall has been detected.
    EVENT_VIRTUAL_WALL = 8,
    //A wall has been detected.
    EVENT_WALL = 9,
    //A cliff has been detected.
    EVENT_CLIFF = 10,
    //A cliff has been detected (left).
    EVENT_LEFT_CLIFF = 11,
    //A cliff has been detected (front left).
    EVENT_FRONT_LEFT_CLIFF = 12,
    //A cliff has been detected (front right).
    EVENT_FRONT_RIGHT_CLIFF = 13,
    //A cliff has been detected (right).
    EVENT_RIGHT_CLIFF = 14,
    //Home base has been detected.
    EVENT_HOME_BASE = 15,
    //Advance button has been pushed.
    EVENT_ADVANCE_BUTTON = 16,
    //Play button has been pushed.
    EVENT_PLAY_BUTTON = 17,
    //Digital input 0 has changed.
    EVENT_DIGITAL_INPUT_0 = 18,
    //Digital input 1 has changed.
    EVENT_DIGITAL_INPUT_1 = 19,
    //Digital input 2 has changed.
    EVENT_DIGITAL_INPUT_2 = 20,
    //Digital input 3 has changed.
    EVENT_DIGITAL_INPUT_3 = 21,
    //Robot has switched to passive mode.
    EVENT_OI_MODE_PASSIVE = 22
  };

  //Indicate an event state.
  enum EventState {
    //The event is happening currently.
    EVENT_OCCURRING,
    //The event is not happening currently.
    EVENT_NOT_OCCURRING
  };

  //Enumerate available opcodes.
  //Opcode are the basics instructions that the protocol
  //support. This class wraps them to avoid using them directly.
  //The only case where you have to use them is when you want
  //to send an Open Interface script.
  enum Opcode {
    //Start command.
    OPCODE_START = 128,
    //Baud command.
    OPCODE_BAUD = 129,
    //Control command (equivalent to safe).
    OPCODE_CONTROL = 130,
    //Safe command.
    OPCODE_SAFE = 131,
    //Full command.
    OPCODE_FULL = 132,
    //Spot demo command.
    OPCODE_SPOT = 134,
    //Cover demo command.
    OPCODE_COVER = 135,
    //Demo command.
    OPCODE_DEMO = 136,
    //Driver command.
    OPCODE_DRIVE = 137,
    //Low side drivers command.
    OPCODE_LOW_SIDE_DRIVERS = 138,
    //Leds command.
    OPCODE_LEDS = 139,
    //Song command.
    OPCODE_SONG = 140,
    //Play song command.
    OPCODE_PLAY = 141,
    //Sensors command.
    OPCODE_SENSORS = 142,
    //Cover and dock demo command.
    OPCODE_COVER_AND_DOCK = 143,
    //Pwm low side drivers command.
    OPCODE_PWM_LOW_SIDE_DRIVERS = 144,
    //Driver direct command.
    OPCODE_DRIVE_DIRECT = 145,
    //Digital ouputs command.
    OPCODE_DIGITAL_OUTPUTS = 147,
    //Stream command.
    OPCODE_STREAM = 148,
    //Query list command.
    OPCODE_QUERY_LIST = 149,
    //Pause/resume stream command.
    OPCODE_PAUSE_RESUME_STREAM = 150,
    //Send IR command.
    OPCODE_SEND_IR = 151,
    //Script command.
    OPCODE_SCRIPT = 152,
    //Play script command.
    OPCODE_PLAY_SCRIPT = 153,
    //Show script command.
    OPCODE_SHOW_SCRIPT = 154,
    //Wait time command.
    OPCODE_WAIT_TIME = 155,
    //Wait distance command.
    OPCODE_WAIT_DISTANCE = 156,
    //Wait angle command.
    OPCODE_WAIT_ANGLE = 157,
    //Wait event command.
    OPCODE_WAIT_EVENT = 158,
    OPCODE_UNKNOWN_159 = 159,
    OPCODE_UNKNOWN_160 = 160,
    OPCODE_UNKNOWN_161 = 161,
    OPCODE_UNKNOWN_162 = 162
  };

  //Enumerate available sensor packets.
  enum SensorPacket {
    //Groups packets 7 to 26.
    SENSOR_GROUP_0 = 0,
    //Groups packets 7 to 16.
    SENSOR_GROUP_1 = 1,
    //Groups packets 17 to 20.
    SENSOR_GROUP_2 = 2,
    //Groups packets 21 to 26.
    SENSOR_GROUP_3 = 3,
    //Groups packets 27 to 34.
    SENSOR_GROUP_4 = 4,
    //Groups packets 35 to 42.
    SENSOR_GROUP_5 = 5,
    //Groups packets 7 to 42.
    SENSOR_GROUP_6 = 6,
    //Wheel and bumper states.
    SENSOR_BUMPS_WHEELS_DROPS = 7,
    //Wall sensor state.
    SENSOR_WALL = 8,
    //Left cliff sensor state.
    SENSOR_CLIFF_LEFT = 9,
    //Front left cliff sensor state.
    SENSOR_CLIFF_FRONT_LEFT = 10,
    //Front right cliff sensor state.
    SENSOR_CLIFF_FRONT_RIGHT = 11,
    //Right cliff sensor state.
    SENSOR_CLIFF_RIGHT = 12,
    //Virtual wall sensor state.
    SENSOR_VIRTUAL_WALL = 13,
    //Overcurrent sensors states.
    SENSOR_OVERCURRENTS = 14,
    UNUSED_15 = 15,
    UNUSED_16 = 16,
    //IR bytes received.
    SENSOR_IR = 17,
    //Buttons states.
    SENSOR_BUTTONS = 18,
    //Traveled distance since last read.
    SENSOR_DISTANCE = 19,
    //Turned angle since last read.
    SENSOR_ANGLE = 20,
    //Charging state.
    SENSOR_CHARGING_STATE = 21,
    //Battery voltage.
    SENSOR_VOLTAGE = 22,
    //Battery current.
    SENSOR_CURRENT = 23,
    //Battery temperature.
    SENSOR_BATTERY_TEMPERATURE = 24,
    //Battery charge in milliamp-hours (mAh).
    SENSOR_BATTERY_CHARGE = 25,
    //Battery charge capacity in milliamp-hours (mAh).
    SENSOR_BATTERY_CAPACITY = 26,
    //Wall's sensor signal strength.
    SENSOR_WALL_SIGNAL = 27,
    //Left cliff signal strength.
    SENSOR_CLIFF_LEFT_SIGNAL = 28,
    //Front left cliff signal strength.
    SENSOR_CLIFF_FRONT_LEFT_SIGNAL = 29,
    //Front right cliff signal strength.
    SENSOR_CLIFF_FRONT_RIGHT_SIGNAL = 30,
    //Right cliff signal strength.
    SENSOR_CLIFF_RIGHT_SIGNAL = 31,
    //Cargo Bay digital input strength.
    SENSOR_CARGO_BAY_DIGITAL_INPUT = 32,
    //Cargo Bay analog input strength.
    SENSOR_CARGO_BAY_ANALOG_SIGNAL = 33,
    //Available charging sources.
    SENSOR_CHARGING_SOURCES_AVAILABLE = 34,
    //Current Open Interface mode.
    SENSOR_OI_MODE = 35,
    //Current selected song.
    SENSOR_SONG_NUMBER = 36,
    //Indicates whether or not a song is being played.
    SENSOR_SONG_PLAYING = 37,
    //List of streamed packets.
    SENSOR_NUMBER_STREAM_PACKETS = 38,
    //Requested velocity.
    SENSOR_REQUESTED_VELOCITY = 39,
    //Requested radius.
    SENSOR_REQUESTED_RADIUS = 40,
    //Requested right velocity.
    SENSOR_REQUESTED_RIGHT_VELOCITY = 41,
    //Requested left velocity.
    SENSOR_REQUESTED_LEFT_VELOCITY = 42
  };

  //Define the green color for power led.
  static const unsigned char LED_COLOR_GREEN = 0;
  //Define the red color for power led.
  static const unsigned char LED_COLOR_RED = 255;

  //Define minimum intensity for power led (off).
  static const unsigned char LED_INTENSITY_OFF = 0;
  //Define full intensity for power led.
  static const unsigned char LED_INTENSITY_FULL = 255;

  //Define minimum velocity for low side drivers.
  static const unsigned char LOW_SIDE_VELOCITY_MIN = 0;

  //Define maximum velocity for low side drivers.
  static const unsigned char LOW_SIDE_VELOCITY_MAX = 128;

  //Define minimum velocity for robot wheels motors.
  static const int VELOCITY_MIN = -500;
  //Define maximum velocity for robot wheels motors.
  static const int VELOCITY_MAX = 500;

  //Define minimum radius turn of the robot.
  static const int RADIUS_MIN = -2000;
  //Define maximum radius turn of the robot.
  static const int RADIUS_MAX = 2000;

  //Define minimum song id.
  static const unsigned char SONG_MIN = 0;
  //Define maximum song id.
  static const unsigned char SONG_MAX = 15;

  //Define song maximum size.
  static const unsigned char SONG_MAX_SIZE = 16;

  //Define the value for a rest node (no sound).
  static const unsigned char NO_NOTE = 30;
  //Define the minimum note (G).
  static const unsigned char NOTE_MIN = 31;
  //Define the maximum note (G).
  static const unsigned char NOTE_MAX = 127;

  //Define maximum script size.
  static const unsigned char SCRIPT_MAX_SIZE = 100;

  //Stream header ``magic value''.
  static const unsigned char STREAM_HEADER = 19;

  //Bit used to retrieve the wheeldrop caster status.
  static const unsigned char SENSOR_BIT_WHEELDROP_CASTER = 4;
  //Bit used to retrieve the wheeldrop caster status.
  static const unsigned char SENSOR_BIT_WHEELDROP_LEFT = 3;
  //Bit used to retrieve the wheeldrop caster status.
  static const unsigned char SENSOR_BIT_WHEELDROP_RIGHT = 2;
  //Bit used to retrieve the bump left status.
  static const unsigned char SENSOR_BIT_BUMP_LEFT = 1;
  //Bit used to retrieve the bump right status.
  static const unsigned char SENSOR_BIT_BUMP_RIGHT = 0;

  //Bit used to retrieve the left wheel overcurrent status.
  static const unsigned char SENSOR_BIT_LEFTWHEELOVERCURRENT = 4;
  //Bit used to retrieve the right wheel overcurrent status.
  static const unsigned char SENSOR_BIT_RIGHTWHEELOVERCURRENT = 3;
  //Bit used to retrieve the LD2 overcurrent status.
  static const unsigned char SENSOR_BIT_LD2OVERCURRENT = 2;
  //Bit used to retrieve the LD1 overcurrent status.
  static const unsigned char SENSOR_BIT_LD1OVERCURRENT = 1;
  //Bit used to retrieve the LD0 overcurrent status.
  static const unsigned char SENSOR_BIT_LD0OVERCURRENT = 0;

  //Bit used to retrieve the advance button status.
  static const unsigned char SENSOR_BIT_ADVANCEBUTTON = 2;
  //Bit used to retrieve the play button status.
  static const unsigned char SENSOR_BIT_PLAYBUTTON = 0;

  //Bit used to retrieve the device detect pin status.
  static const unsigned char SENSOR_BIT_DEVICEDETECT = 4;
  //Bit used to retrieve the digital input 3 pin status.
  static const unsigned char SENSOR_BIT_DIGITALINPUT3 = 3;
  //Bit used to retrieve the digital input 2 pin status.
  static const unsigned char SENSOR_BIT_DIGITALINPUT2 = 2;
  //Bit used to retrieve the digital input 1 pin status.
  static const unsigned char SENSOR_BIT_DIGITALINPUT1 = 1;
  //Bit used to retrieve the digital input 0 pin status.
  static const unsigned char SENSOR_BIT_DIGITALINPUT0 = 0;

  //Bit used to retrieve whether the home base charger is available or not.
  static const unsigned char SENSOR_BIT_HOMEBASECHARGERAVAILABLE = 1;
  //Bit used to retrieve whether the internal charger is available or not.
  static const unsigned char SENSOR_BIT_INTERNALCHARGERAVAILABLE = 0;

private:
  OpenInterface();
  virtual ~OpenInterface();
};
}  //namespace create

#endif //ENUMS_H_
