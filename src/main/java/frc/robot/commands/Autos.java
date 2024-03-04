// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command auto(Drivebase drivebase, String autoNumber, String initial) {
    int length = autoNumber.length();
    Command runAutoCommand = new ShootCmd();
    char pre = '0';

    switch (initial) {
      case "left":
        drivebase.resetPose(AutoConstants.leftPose2d);
        break;
      case "middle":
        drivebase.resetPose(AutoConstants.middlePose2d);
        break;
      case "right":
        drivebase.resetPose(AutoConstants.rightPose2d);
        break;
    }

    for (int i = 0; i < length; i++) {
      char cur = autoNumber.charAt(i);
      if (pre == '0' || pre == '1' || pre == '2' || pre == '3') {
        switch (cur) {
          case '1':
            runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 6.68, 25.53, AutoConstants.kMaxVelocity,
                AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0, 0));
            break;
          case '2':
            runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 5.5, 0, AutoConstants.kMaxVelocity,
                AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0, 0));
            break;
          case '3':
            runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 4.4, -29.16, AutoConstants.kMaxVelocity,
                AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0, 0));
            break;
          case '4':
            runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RTSToNote4,
                AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0.0));
            break;
          case '5':
            runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RTSToNote5,
                AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0.0));
            break;
          case '6':
            runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote6,
                AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0.0));
            break;
          case '7':
            runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote7,
                AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0.0));
            break;
          case '8':
            runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote8,
                AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
                AutoConstants.kMaxAngularAcceleration, 0.0));
            break;
        }

      } else if (pre == '4' || pre == '5' || pre == '6') {
        switch (cur) {
          case '1':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote1));
            break;
          case '2':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote2));
            break;
          case '3':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote3));
            break;
          case '4':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote4));
            break;
          case '5':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote5));
            break;
          case '6':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote6));
            break;
          case '7':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote7));
            break;
          case '8':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote8));
            break;
        }
      } else if (pre == '7' || pre == '8') {
        switch (cur) {
          case '1':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote1));
            break;
          case '2':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote2));
            break;
          case '3':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote3));
            break;
          case '4':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote4));
            break;
          case '5':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote5));
            break;
          case '6':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote6));
            break;
          case '7':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote7));
            break;
          case '8':
            runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote8));
            break;
        }
      }

      runAutoCommand.andThen(new IntakeCmd());

      switch (cur) {
        case '1':
          // aim
          runAutoCommand.andThen(new ShootCmd());
          break;
        case '2':
          // aim
          runAutoCommand.andThen(new ShootCmd());
          break;
        case '3':
          // aim
          runAutoCommand.andThen(new ShootCmd());
          break;
        case '4':
          // aim
          runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
              AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
              AutoConstants.kMaxAngularAcceleration, 0.0)).andThen(new ShootCmd());
          break;
        case '5':
          // aim
          runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
              AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
              AutoConstants.kMaxAngularAcceleration, 0.0)).andThen(new ShootCmd());
          break;
        case '6':
          // aim
          runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
              AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
              AutoConstants.kMaxAngularAcceleration, 0.0)).andThen(new ShootCmd());
          break;
        case '7':
          // aim
          runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.bottomRelayToRBS,
              AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
              AutoConstants.kMaxAngularAcceleration, 0.0)).andThen(new ShootCmd());
          break;
        case '8':
          // aim
          runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.bottomRelayToRBS,
              AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
              AutoConstants.kMaxAngularAcceleration, 0.0)).andThen(new ShootCmd());
          break;
      }

      pre = cur;
    }

    return runAutoCommand;
  }

  // public static Command exampleAuto() {
  // return Commands.sequence();
  // }

  // public static Command goStraightFroward(Drivebase drivebase) {
  // return drivebase.followPathCommand(AutoConstants.pathGoStraightForward);
  // }

  // public static Command turnRight(Drivebase drivebase) {
  // return drivebase.followPathCommand(AutoConstants.pathTurnRight);
  // }

  // public static Command goStraightFrowardAndTurnRight(Drivebase drivebase) {
  // return
  // drivebase.followAutoCommand(AutoConstants.autoGoStraightFrowardAndTurnRight);
  // }

  // public static Command choreoGoStraightForward(Drivebase drivebase) {
  // return drivebase.followChoreoCommand(AutoConstants.choreoGoStraightForward);
  // }

  // public static Command pathOnTheFly (Drivebase drivebase, AutoPathSubsystem
  // autoPathSubsystem) {
  // return Commands.none();
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}