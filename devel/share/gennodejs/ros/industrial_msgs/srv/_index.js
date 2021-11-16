
"use strict";

let StartMotion = require('./StartMotion.js')
let StopMotion = require('./StopMotion.js')
let SetDrivePower = require('./SetDrivePower.js')
let CmdJointTrajectory = require('./CmdJointTrajectory.js')
let SetRemoteLoggerLevel = require('./SetRemoteLoggerLevel.js')
let GetRobotInfo = require('./GetRobotInfo.js')

module.exports = {
  StartMotion: StartMotion,
  StopMotion: StopMotion,
  SetDrivePower: SetDrivePower,
  CmdJointTrajectory: CmdJointTrajectory,
  SetRemoteLoggerLevel: SetRemoteLoggerLevel,
  GetRobotInfo: GetRobotInfo,
};
