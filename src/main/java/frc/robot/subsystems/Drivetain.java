// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetainConstants;
import frc.robot.Constants.ModuleConstants;

public class Drivetain extends SubsystemBase {
  /** Creates a new Drivetain. */
  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;

  private final SwerveDriveOdometry odometry;

  private final AHRS gyro;

  private final PIDController rotController = new PIDController(0.5, 0, 0);

  public Drivetain() {
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(10, 11, 5, DrivetainConstants.kFrontLeftDriveMotorInverted);
    frontRight = new SwerveModule(15, 14, 4, DrivetainConstants.kFrontRightDriveMotorInverted);
    backLeft = new SwerveModule(12, 13, 2, DrivetainConstants.kBackLeftDriveMotorInverted);
    backRight = new SwerveModule(17, 16, 3, DrivetainConstants.kBackRightDriveMotorInverted);

    SmartDashboard.putNumber("MaxTurningVoltage", ModuleConstants.kMaxModuleTuringVoltage);
    SmartDashboard.putNumber("kD", ModuleConstants.kDRotController);

    gyro = new AHRS(Port.kMXP);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // create the odometry
    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    // reset the gyro
    setGyroReset();

    AutoBuilder.configureHolonomic(
        this::getPose2d, // Robot pose suppier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void setGyroReset() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetainConstants.kMaxSpeed);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("frontLeft_speed", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight_speed", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft_speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRight_speed", swerveModuleStates[3].speedMetersPerSecond);
  }

  public void testDrive(double driveSpd, double rotSpd) {
    // frontLeft.setMotorPower(driveSpd, rotSpd);
    // frontRight.setMotorPower(driveSpd, rotSpd);
    // backLeft.setMotorPower(driveSpd, rotSpd);
    backRight.setMotorPower(driveSpd, rotSpd);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
    // ModuleConstants.kDRotController = SmartDashboard.getNumber("kD",
    // ModuleConstants.kDRotController);

  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  // by test the different xboxController mode
  public double PIDcontrolRot(double rightX, double rightY) {
    double angle = Math.atan2(rightY, rightX);
    double rotSpd = rotController.calculate(gyro.getRotation2d().getRadians(), angle);
    return rotSpd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }


  // auto drive
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
        path,
        this::getPose2d, // Robot pose supplier
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }
}