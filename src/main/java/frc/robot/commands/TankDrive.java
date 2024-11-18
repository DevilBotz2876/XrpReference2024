// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class TankDrive extends Command {
  private final Drive drive;
  private final Supplier<Double> leftSpeedSupplier;
  private final Supplier<Double> rightRotateSupplier;

  public TankDrive(
      Drive drive, Supplier<Double> leftSpeedSupplier, Supplier<Double> rightRotateSupplier) {
    this.drive = drive;
    this.leftSpeedSupplier = leftSpeedSupplier;
    this.rightRotateSupplier = rightRotateSupplier;
    addRequirements((Subsystem) drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.tankDrive(
        leftSpeedSupplier.get() * DriveConstants.maxLinearVelocityMetersPerSec,
        rightRotateSupplier.get() * DriveConstants.maxLinearVelocityMetersPerSec);
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
