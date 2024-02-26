// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.AutoPathSubsystem;
import frc.robot.subsystems.Drivebase;

public final class Autos {
  /** Example static factory for an autonomous command. */
  Drivebase drivebase = new Drivebase();

  public static Command exampleAuto() {
    return Commands.sequence();
  }

  public static Command goStraightFroward(Drivebase drivebase) {
    return drivebase.followPathCommand(AutoConstants.pathGoStraightForward);
  }

  public static Command turnRight(Drivebase drivebase) {
    return drivebase.followPathCommand(AutoConstants.pathTurnRight);
  }

  // public static Command goStraightFrowardAndTurnRight(Drivebase drivebase) {
  //   return drivebase.followAutoCommand(AutoConstants.autoGoStraightFrowardAndTurnRight);
  // }

  public static Command choreoGoStraightForward(Drivebase drivebase) {
    return drivebase.followChoreoCommand(AutoConstants.choreoGoStraightForward);
  }

  public static Command pathOnTheFly (Drivebase drivebase, AutoPathSubsystem autoPathSubsystem) {
    return Commands.none();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}