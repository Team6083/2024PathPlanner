// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetainConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANcoder turningEncoder;

  private final RelativeEncoder driveEncoder;

  private final PIDController rotController;

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannelA, boolean driveInverted) {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    turningEncoder = new CANcoder(turningEncoderChannelA);
    // turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);

    driveEncoder = driveMotor.getEncoder();

    driveMotor.setInverted(driveInverted);
    turningMotor.setInverted(true);

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 150);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 150);
    driveMotor.setClosedLoopRampRate(ModuleConstants.kClosedLoopRampRate);
    turningMotor.setSmartCurrentLimit(20);
    driveMotor.setClosedLoopRampRate(0.25);

    driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
    turningMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

    rotController = new PIDController(ModuleConstants.kPRotController, 0, ModuleConstants.kDRotController);
    rotController.enableContinuousInput(-180.0, 180.0);

    configDriveMotor();
    configTurningMotor();
    driveMotor.burnFlash();
    turningMotor.burnFlash();
    SmartDashboard.putNumber("degree", 0);
  }

  public void configDriveMotor() {
    driveMotor.clearFaults();
    driveMotor.enableVoltageCompensation(ModuleConstants.kMaxModuleDriveVoltage);
    driveEncoder.setPosition(0);
  }

  public void configTurningMotor(){
    turningMotor.clearFaults();
  }


  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // to get the single swerveModule speed and the turning rate
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate(), new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition().getValueAsDouble())));
  }

  // to get the drive distance
  public double getDriveDistance() {
    return driveEncoder.getPosition() / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() / 60.0 / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // to get rotation of turning motor
  public double getRotation() {
    return turningEncoder.getAbsolutePosition().getValueAsDouble();
  }

  // to the get the postion by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition().getValueAsDouble())));
  }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(goalState,
        Rotation2d.fromDegrees(currentTurningDegree));
    double driveMotorVoltage = ModuleConstants.kDesireSpeedtoMotorVoltage * optimizedState.speedMetersPerSecond;
    double turningMotorVoltage = rotController.calculate(currentTurningDegree, optimizedState.angle.getDegrees());
    double[] moduleState = { driveMotorVoltage, turningMotorVoltage };
    return moduleState;
  }

  public double checkOverVoltage(double currentVoltage, double goalVoltage){
    double error = goalVoltage-currentVoltage;
    if(Math.abs(goalVoltage)<DrivetainConstants.kMinSpeed*ModuleConstants.kDesireSpeedtoMotorVoltage){
      return goalVoltage;
    }
    if(Math.abs(error)>ModuleConstants.kLimitModuleDriveVoltage){
      error *= ModuleConstants.kLimitModuleDriveVoltage/ModuleConstants.kMaxModuleDriveVoltage;
      return currentVoltage+error;
    }else{
      return goalVoltage;
    }
    
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DrivetainConstants.kMinSpeed) {
      stopModule();
    } else {
      var moduleState = optimizeOutputVoltage(desiredState, getRotation());
      // moduleState[0] = checkOverVoltage(driveMotor.getOutputCurrent(), moduleState[0]);
      driveMotor.setVoltage(moduleState[0]);
      turningMotor.setVoltage(moduleState[1]);
      SmartDashboard.putNumber(turningEncoder+"_voltage", moduleState[0]);
    }
  }

  // for one module test
  public void setMotorPower(double driveSpd, double rotSpd) {
    driveMotor.set(driveSpd);
    turningMotor.set(rotSpd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(turningEncoder+"_current", driveMotor.getOutputCurrent());
    SmartDashboard.putBoolean(turningEncoder+"_overcurrent", 
    driveMotor.getFault(FaultID.kOvercurrent));
  }

}
