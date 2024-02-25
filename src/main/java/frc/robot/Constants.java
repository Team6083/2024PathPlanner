// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class DrivebaseConstants {
    // drive motor channel
    public static final int kFrontLeftDriveMotorChannel = 11;
    public static final int kFrontRightDriveMotorChannel = 15;
    public static final int kBackLeftDriveMotorChannel = 13;
    public static final int kBackRightDriveMotorChannel = 17;

    // turning motor channel
    public static final int kFrontLeftTurningMotorChannel = 12;
    public static final int kFrontRightTurningMotorChannel = 16;
    public static final int kBackLeftTurningMotorChannel = 14;
    public static final int kBackRightTurningMotorChannel = 18;

    // turnning encoder channel
    public static final int kFrontLeftTurningEncoderChannel = 31;
    public static final int kFrontRightTurningEncoderChannel = 32;
    public static final int kBackLeftTurningEncoderChannel = 33;
    public static final int kBackRightTurningEncoderChannel = 34;

    // // can coder magnet offset value
    public static final double kFrontLeftCanCoderMagOffset = -0.087891;
    public static final double kFrontRightCanCoderMagOffset = -0.457764;
    public static final double kBackLeftCanCoderMagOffset = 0.357178;
    public static final double kBackRightCanCoderMagOffset = -0.345703;
    // public static final double kFrontLeftCanCoderMagOffset = -0.079590;
    // public static final double kFrontRightCanCoderMagOffset = -0.458984;
    // public static final double kBackLeftCanCoderMagOffset = 0.355225;
    // public static final double kBackRightCanCoderMagOffset = -0.333984 ;

    public static final double kMaxSpeed = 5;
    public static final double kMinSpeed = 0.25;
    public static final double kMinJoyStickValue = 0.3;
    public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

    public static final double xLimiterRateLimit = 3.0;
    public static final double yLimiterRateLimit = 3.0;
    public static final double rotLimiterRateLimit = 3.0;

    public static final boolean kFrontLeftDriveMotorInverted = true;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kBackLeftDriveMotorInverted = true;
    public static final boolean kBackRightDriveMotorInverted = false;

    public static final boolean kGyroInverted = false; // wheather gyro is under the robot

    public static final double kGyroOffSet = 0;
  }

  public static final class ModuleConstants {
    public static final double kWheelRadius = 0.046;

    public static final double kWheelDiameterMeters = 0.15;

    public static final double kMaxModuleDriveVoltage = 12.0;

    public static final double kDriveClosedLoopRampRate = 0;// 1 second 1 unit
    public static final double kTurningClosedLoopRampRate = 0.25;

    public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivebaseConstants.kMaxSpeed;

    public static final double kMaxModuleTuringVoltage = 5.0;

    public static final double kMaxSpeedTurningDegree = 180.0;

    public static final double kPRotController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;
    public static final double kIRotController = 0.0;
    public static final double kDRotController = 0.0004;

    public static final boolean kTurningMotorInverted = true;
  }

  public static final class AutoConstants {
    public static final String pathGoStraightForward = "GoStraightForward";
    public static final String pathTurnRight = "TurnRight";

    // public static final String autoGoStraightFrowardAndTurnRight = "New Auto";

    public static final String choreoGoStraightForward = "goStraightForward";

    public static final double kPTranslation = 2.0;
    public static final double kITranslation = 0.001;
    public static final double kDTranslation = 0.01;
    public static final double kPRotation = 1.5;
    public static final double kIRotation = 0.0;
    public static final double kDRotation = 0.0;
    public static final double maxModuleSpeed = 2.5;
    public static final double drivebaseRadius = 2.17;
  }
}
