// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.EncoderResetCmd;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.PoseResetCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Drivebase;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  Drivebase drivebase;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // private final PowerDistribution pd = new PowerDistribution();

  private SendableChooser<Command> autoChooser;
  private SendableChooser<String> initialChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivebase = new Drivebase();
    drivebase.init();

    NamedCommands.registerCommand("null", new WaitCommand(0.5));
    NamedCommands.registerCommand("GyroReset", new GyroResetCmd(drivebase));
    // NamedCommands.registerCommand("none", null);

    // Configure the trigger bindings

    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = new SendableChooser<Command>();
    autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser = new SendableChooser<Command>();

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    // autoChooser.addOption("Forward", Autos.goStraightFroward(drivebase));
    // autoChooser.addOption("TurnRight", Autos.turnRight(drivebase));
    // autoChooser.addOption("Combine",
    // Autos.goStraightFrowardAndTurnRight(drivebase));
    // autoChooser.addOption(("Choreo Forward"),
    // Autos.choreoGoStraightForward(drivebase));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    initialChooser = new SendableChooser<String>();
    initialChooser.setDefaultOption("none", null);
    initialChooser.addOption("left", "left");
    initialChooser.addOption("middle", "middle");
    initialChooser.addOption("right", "right");
    SmartDashboard.putString("auto", null);
    SmartDashboard.putData(initialChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, driverController));
    var gyroResetCmd = new GyroResetCmd(drivebase);
    var poseResetCmd = new PoseResetCmd(drivebase);
    driverController.back().onTrue(gyroResetCmd);
    driverController.start().onTrue(poseResetCmd);
    // driverController.y().onTrue(new EncoderResetCmd(drivebase));

    SmartDashboard.putBoolean("reset_pose", false);
    SmartDashboard.putBoolean("reset_gyro", false);
    SmartDashboard.putData(gyroResetCmd);
    SmartDashboard.putData(poseResetCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoNumber = SmartDashboard.getString("auto", null);
    String initial = initialChooser.getSelected();
    var alliance = DriverStation.getAlliance();
    if (initial == null && alliance.isPresent())
      return new InstantCommand();
    Boolean isRed = alliance.get() == DriverStation.Alliance.Red;
      if (isRed) {
      initial = (initial == "left" ? "right" : (initial == "right" ? "left" : "middle"));
    }
    return Autos.auto(drivebase, autoNumber, initial);
  }
}
