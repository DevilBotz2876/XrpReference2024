package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.arm.Arm;
import java.util.function.Supplier;

public class SetArmAngleCommand extends Command {
  private final Arm arm;
  private final Supplier<Double> angle;

  public SetArmAngleCommand(Arm arm, Supplier<Double> angle) {
    this.arm = arm;
    this.angle = angle;
    addRequirements((Subsystem) arm);
    System.out.println(angle);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setAngle(angle.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
