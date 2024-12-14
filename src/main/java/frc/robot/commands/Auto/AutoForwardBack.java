package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.drive.Drive;

public class AutoForwardBack extends SequentialCommandGroup {

  public AutoForwardBack(Drive drive) {
    addCommands(
        new ArcadeDrive(drive, () -> 1.0, () -> 0.0).withTimeout(1));
       
  }
}

