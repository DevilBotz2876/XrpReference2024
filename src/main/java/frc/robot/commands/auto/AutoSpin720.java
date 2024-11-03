package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.TurnDegrees;
import frc.robot.subsystems.drive.Drive;

public class AutoSpin720 extends SequentialCommandGroup {

  public AutoSpin720(Drive drive) {
    addCommands(
        new TurnDegrees(drive, 180, 0.25).withTimeout(1.5),
        new TurnDegrees(drive, 180, 0.25).withTimeout(1.5),
        new TurnDegrees(drive, 180, 0.25).withTimeout(1.5),
        new TurnDegrees(drive, 180, 0.25).withTimeout(1.5));
  }
}
