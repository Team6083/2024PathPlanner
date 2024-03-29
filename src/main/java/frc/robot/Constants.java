// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
  public static class FieldConstants {
    public static final double speakerFrontTall = Units.feetToMeters(6) + Units.inchesToMeters(11); // 210.82cm
    public static final double speakerBackTall = Units.feetToMeters(6) + Units.inchesToMeters(6); // 198.12cm
    public static final double speakerWidth = Units.feetToMeters(3) + Units.inchesToMeters(5.675); // 105.8545cm
    public static final double speakerExtend = Units.inchesToMeters(18); // 45.72cm
  }

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
    public static final double kFrontLeftCanCoderMagOffset = -0.076416;
    public static final double kFrontRightCanCoderMagOffset = -0.464355;
    public static final double kBackLeftCanCoderMagOffset = 0.360107;
    public static final double kBackRightCanCoderMagOffset = -0.342285;

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

    public static final String LBSToNote1 = "LBSToNote1";
    public static final String LBSToNote2 = "LBSToNote2";
    public static final String LBSToNote3 = "LBSToNote3";
    public static final String LBSToNote4 = "LBSToNote4";
    public static final String LBSToNote5 = "LBSToNote5";
    public static final String LBSToNote6 = "LBSToNote6";
    public static final String LBSToNote7 = "LBSToNote7";
    public static final String LBSToNote8 = "LBSToNote8";

    public static final String LTSToNote1 = "LTSToNote1";
    public static final String LTSToNote2 = "LTSToNote2";
    public static final String LTSToNote3 = "LTSToNote3";
    public static final String LTSToNote4 = "LTSToNote4";
    public static final String LTSToNote5 = "LTSToNote5";
    public static final String LTSToNote6 = "LTSToNote6";
    public static final String LTSToNote7 = "LTSToNote7";
    public static final String LTSToNote8 = "LTSToNote8";

    public static final String RBSToNote1 = "RBSToNote1";
    public static final String RBSToNote2 = "RBSToNote2";
    public static final String RBSToNote3 = "RBSToNote3";
    public static final String RBSToNote4 = "RBSToNote4";
    public static final String RBSToNote5 = "RBSToNote5";
    public static final String RBSToNote6 = "RBSToNote6";
    public static final String RBSToNote7 = "RBSToNote7";
    public static final String RBSToNote8 = "RBSToNote8";

    public static final String RTSToNote1 = "RTSToNote1";
    public static final String RTSToNote2 = "RTSToNote2";
    public static final String RTSToNote3 = "RTSToNote3";
    public static final String RTSToNote4 = "RTSToNote4";
    public static final String RTSToNote5 = "RTSToNote5";
    public static final String RTSToNote6 = "RTSToNote6";
    public static final String RTSToNote7 = "RTSToNote7";
    public static final String RTSToNote8 = "RTSToNote8";

    public static final String bottomRelayToLBS = "bottomRelayToLBS";
    public static final String bottomRelayToRBS = "bottomRelayToRBS";
    public static final String topRelayToLTS = "topRelayToLTS";
    public static final String topRelayToRTS = "topRelayToRTS";

    public static final double kPTranslation = 2.0;
    public static final double kITranslation = 0.001;
    public static final double kDTranslation = 0.01;
    public static final double kPRotation = 2.0;
    public static final double kIRotation = 0.001;
    public static final double kDRotation = 0.01;
    public static final double maxModuleSpeed = 3.36;
    public static final double drivebaseRadius = 2.17;

    public static final double kMaxVelocity = 3.36;
    public static final double kMaxAcceleration = 3.36;
    public static final double kMaxAngularVelocity = 453.38;
    public static final double kMaxAngularAcceleration = 453.38;
    public static final double rotationDelayDistance = 0.0;

    public static final Pose2d leftPose2d = new Pose2d(0.76, 6.53, Rotation2d.fromDegrees(60));
    public static final Pose2d middlePose2d = new Pose2d(1.24, 5.5, Rotation2d.fromDegrees(0));
    public static final Pose2d rightPose2d = new Pose2d(0.76, 4.56, Rotation2d.fromDegrees(-60));
  }
}
