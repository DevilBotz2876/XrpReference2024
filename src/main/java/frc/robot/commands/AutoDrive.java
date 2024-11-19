
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class AutoDrive extends SequentialCommandGroup {

    public AutoDrive(Drive drive) {

        addCommands(new ArcadeDrive(drive, () -> 1.0, () -> 0.0).withTimeout(3));
    }
}