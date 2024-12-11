package frc.robot.commands.auto.center;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Center1Note extends SequentialCommandGroup {

    public Center1Note(Drive drive, Arm arm, Shooter shooter, Intake intake) {
        addCommands(
            // new ParallelCommandGroup(
                new InstantCommand(() -> {
                    arm.setAngle(120);
                }),
                new InstantCommand(() -> {
                    shooter.setSpeed(0.85);
                }),
                // new ShooterCommand(shooter, () -> 0.85, () ->0.0).wit,
            // ),
            new WaitCommand(1),
            new IntakeCommand(intake, () -> 0.5).withTimeout(1.0),
            new InstantCommand(() -> {
                    shooter.setSpeed(0.0);
                }).withTimeout(1.0),
            new TurnDegrees(() -> 1.0, () -> 200.0, drive)
        );
    }
}
