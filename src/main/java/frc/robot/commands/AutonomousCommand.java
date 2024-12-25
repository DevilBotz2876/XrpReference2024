// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystemXrp;

public class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousCommand(DriveSubsystemXrp drivetrain) {

    // Creates a PID Controller for the wheels going in a straight line
    PIDController anglePIDController =
        new PIDController(DriveConstants.anglePIDConstants.kP, 
                          DriveConstants.anglePIDConstants.kI,
                          DriveConstants.anglePIDConstants.kD);

    PIDController distancePidController =
        new PIDController(DriveConstants.distancePIDConstants.kP, 
                          DriveConstants.distancePIDConstants.kI,
                          DriveConstants.distancePIDConstants.kD);

    // Define the drive speed
    double driveSpeed = 1.0; // speed factor

    // Define the target pose
    Pose2d targetPose = new Pose2d(.5, 0.0, new Rotation2d(0.0)); // .5 meters forward, no rotation


    addCommands(
      new DriveToPose(driveSpeed, targetPose, drivetrain, distancePidController, anglePIDController)
      //new DriveDistance(-.5, .2, drivetrain, anglePIDController, distancePidController)
      ); // Drive backward
  }
}