package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmIOXrp implements ArmIO {
  private final XRPServo servo;

  public ArmIOXrp(int deviceNum) {
    servo = new XRPServo(deviceNum);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleDeg = servo.getAngle();
  }

  public void setAngle(double angleDeg) {
    servo.setAngle(angleDeg);
  }
}
