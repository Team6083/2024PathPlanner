// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;

public final class Autos {
  /** Example static factory for an autonomous command. */
  Drivebase drivetain = new Drivebase();

  public static Command exampleAuto() {
    return Commands.sequence();
  }

  public static Command goStraightFroward(Drivebase drivetain) {
    return drivetain.followPathCommand(AutoConstants.pathGoStraightForward);
  }

  public static Command turnRight(Drivebase drivetain) {
    return drivetain.followPathCommand(AutoConstants.pathTurnRight);
  }

  public static Command goStraightFrowardAndTurnRight(Drivebase drivetain) {
    return new PathPlannerAuto("New Path");
    // return PathPlannerAuto.getPathGroupFromAutoFile(AutoConstants.autoGoStraightFrowardAndTurnRight);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}