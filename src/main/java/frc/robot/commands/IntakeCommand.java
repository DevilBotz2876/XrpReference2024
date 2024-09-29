package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public class IntakeCommand extends Command {
  private final Intake intake;
  private final Supplier<Double> speedSupplier;

  public IntakeCommand(Intake intake, Supplier<Double> speedSupplier) {
    this.intake = intake;
    this.speedSupplier = speedSupplier;
    addRequirements((Subsystem) intake);
  }

  @Override
  public void execute() {
    intake.setSpeed(speedSupplier.get());
  }
}
