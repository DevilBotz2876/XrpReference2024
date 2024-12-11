// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
    PIDController wheelPIDController =
        new PIDController(DriveConstants.speedPIDConstants.kP, 
                          DriveConstants.speedPIDConstants.kI,
                          DriveConstants.speedPIDConstants.kD);

    PIDController zAxisPidController =
        new PIDController(DriveConstants.zRotationPIDConstants.kP, 
                          DriveConstants.zRotationPIDConstants.kI,
                          DriveConstants.zRotationPIDConstants.kD);

    addCommands(
        new DriveDistance(1, 1, drivetrain, wheelPIDController, zAxisPidController), // Drive forward
        new DriveDistance(-1, 1, drivetrain, wheelPIDController, zAxisPidController)); // Drive backward
  }
}
