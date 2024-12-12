package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  ArmIO ioOne;  
  ArmIO ioTwo;
  ArmIOInputsAutoLogged inputsOne = new ArmIOInputsAutoLogged();
  ArmIOInputsAutoLogged inputsTwo = new ArmIOInputsAutoLogged();

  public ArmSubsystem(ArmIO ioOne, ArmIO ioTwo) {
    this.ioOne = ioOne;
    this.ioTwo = ioTwo;
  }

  @Override
  public void periodic() {
    ioOne.updateInputs(inputsOne);
    ioTwo.updateInputs(inputsTwo);
    Logger.processInputs("armOne", inputsOne);
    Logger.processInputs("armTwo", inputsTwo);
  }

  public void setAngle(double angleDeg) {
    ioOne.setAngle(angleDeg);
    ioTwo.setAngle(angleDeg);
  }

  public double getAngle() {
    return inputsOne.angleDeg;
  }
}
