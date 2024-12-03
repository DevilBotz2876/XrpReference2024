package frc.robot.subsystems.armtwo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmTwoSubsystem extends SubsystemBase implements ArmTwo {
  ArmTwoIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmTwoSubsystem(ArmTwoIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("arm", inputs);
  }

  public void setAngle(double angleDeg) {
    io.setAngle(angleDeg);
  }

  public double getAngle() {
    return inputs.angleDeg;
  }
}