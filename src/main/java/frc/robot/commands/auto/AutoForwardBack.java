package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.drive.TurnDegrees;
import frc.robot.subsystems.drive.Drive;

public class AutoForwardBack extends SequentialCommandGroup {

  public AutoForwardBack(Drive drive) {
    addCommands(
        new ArcadeDrive(drive, () -> 0.5, () -> 0.0).withTimeout(1),
        new ArcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0.1),
        new TurnDegrees(drive, 180, 0.5),
        new ArcadeDrive(drive, () -> 0.5, () -> 0.0).withTimeout(1),
        new ArcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0.1),
        new TurnDegrees(drive, 180, 0.5));
  }
}
