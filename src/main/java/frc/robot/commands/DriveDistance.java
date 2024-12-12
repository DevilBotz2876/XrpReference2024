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
import org.littletonrobotics.junction.Logger;

public class DriveDistance extends Command {
  private DriveSubsystemXrp drive;
  private double distance;
  private double speed;
  private PIDController distancePIDController;
  private PIDController zRotationPIDController;
  private DifferentialDriveKinematics kinematics;
  private double rightWheelStartPosition;
  private double leftWheelStartPosition;
  private double zRotationStartingPosition;

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
      PIDController dPIDController,
      PIDController zPIDController
      )
       {

    distance = meters;
    speed = driveSpeed;
    drive = driveSubsystem;
    distancePIDController = dPIDController;
    zRotationPIDController = zPIDController; // Initialize the PID controller for Z rotation
    
    // Create a DifferentialDriveKinematics object with the track width
    kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidthMeters);
    


    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    distancePIDController.reset();
    rightWheelStartPosition = drive.getRightPositionMeters();
    leftWheelStartPosition = drive.getLeftPositionMeters();
    zRotationStartingPosition = drive.getAngleZ();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftDistance = leftWheelStartPosition - drive.getLeftPositionMeters();
    double rightDistance = rightWheelStartPosition - drive.getRightPositionMeters();

    double distanceError = leftDistance - rightDistance;

    double distanceCorrection = distancePIDController.calculate(distanceError);

    double zRotationDrift = zRotationStartingPosition - drive.getAngleZ();
    
    // Calculate the Z rotation correction using the PID controller
    double headingCorrection = zRotationPIDController.calculate(zRotationDrift);
    
    double leftSpeed = speed -  distanceCorrection;
    double rightSpeed = speed + distanceCorrection;

    // Create a DifferentialDriveWheelSpeeds object with the wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds =
            new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);

    // // Convert the wheel speeds to chassis speeds
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    // Log the new value
    Logger.recordOutput("leftWheelStartPosition", leftWheelStartPosition);
    Logger.recordOutput("rightWheelStartPosition", rightWheelStartPosition);
    Logger.recordOutput("LeftPositionMeters", drive.getLeftPositionMeters());
    Logger.recordOutput("RightPositionMeters", drive.getLeftPositionMeters());
    Logger.recordOutput("leftDistance", leftDistance);
    Logger.recordOutput("rightDistance", leftDistance);
    Logger.recordOutput("distanceError", distanceError);
    Logger.recordOutput("distanceCorrection", distanceCorrection);
    Logger.recordOutput("zRotationDrift", zRotationDrift);
    Logger.recordOutput("headingCorrection", headingCorrection);
    Logger.recordOutput("leftSpeed", leftSpeed);
    Logger.recordOutput("rightSpeed", rightSpeed);
    Logger.recordOutput("chassisSpeeds", chassisSpeeds);
    Logger.recordOutput("zRotationStartingPosition", zRotationStartingPosition);

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
    return (Math.abs(distanceTraveled) >= distance);
  }
}
