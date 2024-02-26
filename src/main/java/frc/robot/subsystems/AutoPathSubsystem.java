// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoPathSubsystem extends SubsystemBase {
  /** Creates a new AutoPathSubsystem. */
  public AutoPathSubsystem() {

  }

  public void pathOnFly(Pose2d... pose) {
    // Create a list of bezier points from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(pose);

    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a
                                                                 // differential drivetrain, the angular constraints
                                                                 // have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If
                                                           // using a differential drivetrain, the rotation will have no
                                                           // effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
  }

  /**
   * @param x                      Target x
   * @param y                      Target y
   * @param degrees                Target Degrees
   * @param maxVelocity            Max Velociry Mps
   * @param maxAcceleration        Max Acceleration MpsSq
   * @param maxAngularVelocity     Max Angular Velocity Rps
   * @param maxAngularAcceleration Max Angular Acceleration RpsSq
   * @param goalEndVelocity        Goal end velocity in meters/sec
   * @param rotationDelayDistance  Rotation delay distance in meters. This is how
   *                               far the robot should travel before attempting
   *                               to rotate.
   */
  public Command pathFindingToPose(double x, double y, double degrees, double maxVelocity, double maxAcceleration,
      double maxAngularVelocity, double maxAngularAcceleration, double goalEndVelocity,
      double rotationDelayDistance) {

    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration));
    return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocity, rotationDelayDistance);
  }

  /**
   * @param path                   Path Name
   * @param maxVelocity            Max Velociry Mps
   * @param maxAcceleration        Max Acceleration MpsSq
   * @param maxAngularVelocity     Max Angular Velocity Rps
   * @param maxAngularAcceleration Max Angular Acceleration RpsSq
   * @param rotationDelayDistance  Rotation delay distance in meters. This is how
   *                               far the robot should travel before attempting
   *                               to rotate.
   */
  public Command pathFindingThenFollowPath(String pathName, double maxVelocity, double maxAcceleration,
      double maxAngularVelocity, double maxAngularAcceleration, double rotationDelayDistance) {

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAcceleration));
    return AutoBuilder.pathfindThenFollowPath(path, constraints, rotationDelayDistance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
