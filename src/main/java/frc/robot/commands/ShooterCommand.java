package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class ShooterCommand extends Command {
  private final Shooter shooter;
  private final Supplier<Double> ccwSpeedSupplier;
  private final Supplier<Double> cwSpeedSupplier;

  public ShooterCommand(
      Shooter shooter, Supplier<Double> ccwSpeedSupplier, Supplier<Double> cwSpeedSupplier) {
    this.shooter = shooter;
    this.ccwSpeedSupplier = ccwSpeedSupplier;
    this.cwSpeedSupplier = cwSpeedSupplier;
    addRequirements((Subsystem) shooter);
  }

  @Override
  public void execute() {
    if (cwSpeedSupplier.get() > 0) {
      shooter.setSpeed(cwSpeedSupplier.get());
    } else if (ccwSpeedSupplier.get() > 0) {
      shooter.setSpeed(-ccwSpeedSupplier.get());
    } else {
      shooter.setSpeed(0);
    }
  }
}
