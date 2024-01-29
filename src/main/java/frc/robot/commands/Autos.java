// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  Drivetain drivetain = new Drivetain();

  public static Command exampleAuto() {
    return Commands.sequence();
  }

  public static Command goStraightFroward(Drivetain drivetain) {
    return drivetain.followPathCommand(AutoConstants.pathGoStraightForward);
  }

  public static Command turnRight(Drivetain drivetain) {
    return drivetain.followPathCommand(AutoConstants.pathTurnRight);
  }

  public static Command goStraightFrowardAndTurnRight(Drivetain drivetain) {
    return drivetain.followPathCommand(AutoConstants.pathGoStraightForward);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}