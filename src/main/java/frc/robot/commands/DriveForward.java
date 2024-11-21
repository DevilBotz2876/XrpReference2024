package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class DriveForward extends SequentialCommandGroup {
  private final Drive drive;
  private final double distance;

  public DriveForward(Drive drive, double distance) {
    this.drive = drive;
    this.distance = distance;

    addCommands(new ArcadeDrive(drive, () -> 3.0, () -> 0.0).withTimeout(distance * 0.77));
  }
}
