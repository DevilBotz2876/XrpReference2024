// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystemXrp;
import edu.wpi.first.math.controller.PIDController;

public class DriveDistance extends Command {
  private final DriveSubsystemXrp m_drive;
  private final double m_distance;
  private final double m_speed;
  private final PIDController m_pidController;
  

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param meters The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double meters, DriveSubsystemXrp drive) {
    m_distance = meters;
    m_speed = speed;
    m_drive = drive;
    m_pidController = new PIDController(1.0, 0.0, 0.0); // Adjust PID constants as needed
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setChassisSpeeds(0, 0);
    m_drive.resetEncoders();
    m_pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftDistance = m_drive.getLeftPositionMeters();
    double rightDistance = m_drive.getLeftPositionMeters();
    double error = leftDistance - rightDistance;
    double correction = m_pidController.calculate(error);

    double leftSpeed = m_speed - correction;
    double rightSpeed = m_speed + correction;

    m_drive.setChassisSpeeds(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setChassisSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceMeters()) >= m_distance;
  }
}