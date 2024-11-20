// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class TankDrive extends Command {
  private final Drive drive;
  private final Supplier<Double> rightWheelSpeedSupplier;
  private final Supplier<Double> leftWheelSpeedSupplier;

  /**
   * Creates a new TankDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drive The drivetrain subsystem on which this command will run
   * @param leftWheelSpeedSupplier Lambda supplier of forward/backward speed
   * @param rightWheelSpeedSupplier Lambda supplier of rotational speed
   */
  public TankDrive(
      Drive drive, Supplier<Double> leftWheelSpeedSupplier, Supplier<Double> rightWheelSpeedSupplier) {
    this.drive = drive;
    this.rightWheelSpeedSupplier = leftWheelSpeedSupplier;
    this.leftWheelSpeedSupplier = rightWheelSpeedSupplier;
    addRequirements((Subsystem) drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setChassisSpeeds(
         rightWheelSpeedSupplier.get() * DriveConstants.maxLinearVelocityMetersPerSec,
         leftWheelSpeedSupplier.get() * DriveConstants.maxAngularVelocityRadPerSec);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
