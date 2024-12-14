package frc.robot.subsystems.armtwo;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmTwoIOXrp implements ArmTwoIO {
  private final XRPServo servo;

  public ArmTwoIOXrp(int deviceNum) {
    servo = new XRPServo(deviceNum);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleDeg = servo.getAngle();
  }

  public void setAngle(double angleDeg) {
    servo.setAngle(angleDeg);
  }
}
