package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.armtwo.ArmTwo;
import java.util.function.Supplier;

public class ArmTwoCommand extends Command {
  private final ArmTwo arm;
  private final Supplier<Double> angleSupplier;

  public ArmTwoCommand(ArmTwo arm, Supplier<Double> angleSupplier) {
    this.arm = arm;
    this.angleSupplier = angleSupplier;
    addRequirements((Subsystem) arm);
  }

  @Override
  public void execute() {
    arm.setAngle(((angleSupplier.get() + 1) / 2) * 180);
  }
}
