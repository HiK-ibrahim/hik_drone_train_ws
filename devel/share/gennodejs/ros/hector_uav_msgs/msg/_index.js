
"use strict";

let ServoCommand = require('./ServoCommand.js');
let Altimeter = require('./Altimeter.js');
let MotorPWM = require('./MotorPWM.js');
let RawMagnetic = require('./RawMagnetic.js');
let YawrateCommand = require('./YawrateCommand.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let RawRC = require('./RawRC.js');
let Supply = require('./Supply.js');
let MotorCommand = require('./MotorCommand.js');
let ThrustCommand = require('./ThrustCommand.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let RawImu = require('./RawImu.js');
let HeightCommand = require('./HeightCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let MotorStatus = require('./MotorStatus.js');
let ControllerState = require('./ControllerState.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let RuddersCommand = require('./RuddersCommand.js');
let RC = require('./RC.js');
let Compass = require('./Compass.js');

module.exports = {
  ServoCommand: ServoCommand,
  Altimeter: Altimeter,
  MotorPWM: MotorPWM,
  RawMagnetic: RawMagnetic,
  YawrateCommand: YawrateCommand,
  PositionXYCommand: PositionXYCommand,
  RawRC: RawRC,
  Supply: Supply,
  MotorCommand: MotorCommand,
  ThrustCommand: ThrustCommand,
  AttitudeCommand: AttitudeCommand,
  VelocityZCommand: VelocityZCommand,
  RawImu: RawImu,
  HeightCommand: HeightCommand,
  HeadingCommand: HeadingCommand,
  MotorStatus: MotorStatus,
  ControllerState: ControllerState,
  VelocityXYCommand: VelocityXYCommand,
  RuddersCommand: RuddersCommand,
  RC: RC,
  Compass: Compass,
};
