// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystemXrp;

public class DriveDistance extends Command {
  private final DriveSubsystemXrp drive;
  private final double distance;
  private final double speed;
  private final PIDController pidController;
  private final DifferentialDriveKinematics kinematics;
  private double rightWheelStartPosition;
  private double leftWheelStartPosition;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param meters The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(
      double driveSpeed,
      double meters,
      DriveSubsystemXrp driveSubsystem,
      PIDController wheelPIDController) {
    distance = meters;
    speed = driveSpeed;
    drive = driveSubsystem;

    pidController = wheelPIDController; // Adjust PID constants as needed

    // Create a DifferentialDriveKinematics object with the track width
    kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidthMeters);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    pidController.reset();
    rightWheelStartPosition = drive.getRightPositionMeters();
    leftWheelStartPosition = drive.getLeftPositionMeters();

    return;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftDistance = leftWheelStartPosition - drive.getLeftPositionMeters();
    double rightDistance = rightWheelStartPosition - drive.getRightPositionMeters();

    double error = leftDistance - rightDistance;
    double correction = pidController.calculate(error);

    double leftSpeed = speed - correction;
    double rightSpeed = speed + correction;

    // Create a DifferentialDriveWheelSpeeds object with the wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds =
        new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);

    // Convert the wheel speeds to chassis speeds
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    drive.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double leftDistanceTraveled = leftWheelStartPosition - drive.getLeftPositionMeters();
    double rightDistanceTraveled = rightWheelStartPosition - drive.getRightPositionMeters();

    double distanceTraveled = (leftDistanceTraveled + rightDistanceTraveled) / 2.0;
    System.out.println("Distance Traveled: " + distanceTraveled);
    return (Math.abs(distanceTraveled) >= distance);
  }
}
