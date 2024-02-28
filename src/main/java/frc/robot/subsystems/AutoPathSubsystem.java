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

  /**
   * @param maxVelocity            Max Velociry Mps
   * @param maxAcceleration        Max Acceleration MpsSq
   * @param maxAngularVelocity     Max Angular Velocity Rps
   * @param maxAngularAcceleration Max Angular Acceleration RpsSq
   * @param goalEndVelocity        Goal end velocity in meters/sec
   * @param goalEndDegrees         Goal end degrees
   * @param preventFlipping        Prevent the path from being flipped if the
   *                               coordinates are already correct
   * @param pose                   Create a list of bezier points from poses. Each
   *                               pose represents one waypoint.
   *                               The rotation component of the pose should be
   *                               the direction of travel. Do not
   *                               use holonomic rotation.
   */

  public PathPlannerPath pathOnFly(double maxVelocity, double maxAcceleration, double maxAngularVelocity,
      double maxAngularAcceleration, double goalEndVelocity, double goalEndDegrees, boolean preventFlipping,
      Pose2d... pose) {

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(pose);

    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(maxVelocity, maxAcceleration,
            Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration)),
        new GoalEndState(goalEndVelocity, Rotation2d.fromDegrees(goalEndDegrees)));

    path.preventFlipping = preventFlipping;
    return path;
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
    PathConstraints constraints = new PathConstraints(
        maxVelocity, maxAcceleration,
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
    PathConstraints constraints = new PathConstraints(
        maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAcceleration));
    return AutoBuilder.pathfindThenFollowPath(path, constraints, rotationDelayDistance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
