package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class DefaultAuto extends SequentialCommandGroup {
  public DefaultAuto (Drive drive) {
    addCommands(
        // new DriveForward(drive, 1),
        // new DriveTurn(drive, -30), // neg is right pos is left
        new DriveStraight(drive, 5)
    );
  }
}
