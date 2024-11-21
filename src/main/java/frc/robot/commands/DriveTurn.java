package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class DriveTurn extends SequentialCommandGroup {
  private final Drive drive;
  private final double angVel;

  public DriveTurn(Drive drive, double angVel) {
    this.drive = drive;
    this.angVel = angVel;

    addCommands(new ArcadeDrive(drive, () -> 1.0, () -> angVel).withTimeout(0.5));
  }
}
