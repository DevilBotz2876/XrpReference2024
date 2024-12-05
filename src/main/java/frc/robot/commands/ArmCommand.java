package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.arm.Arm;
import java.util.function.Supplier;

public class ArmCommand extends Command {
  private final Arm arm;
  private final Supplier<Double> angleSupplier;

  public ArmCommand(Arm arm, Supplier<Double> angleSupplier) {
    this.arm = arm;
    this.angleSupplier = angleSupplier;
    addRequirements((Subsystem) arm);
  }

  @Override
  public void execute() {
    arm.setAngle(((angleSupplier.get() * 90 + 40)));
  }
}
