package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmSubsystem(ArmIO io) {
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
