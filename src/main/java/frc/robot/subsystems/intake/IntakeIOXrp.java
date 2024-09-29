package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class IntakeIOXrp implements IntakeIO {
  private final XRPServo servo;

  public IntakeIOXrp(int deviceNum) {
    servo = new XRPServo(deviceNum);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speed = servo.getPosition();
  }

  public void setSpeed(double speed) {
    // We need to scale the desired speed [-1.0 .. 1.0] to the servo "position" [0.0 .. 1.0], where
    // "0.5" is stopped for a continuous servo
    servo.setPosition((speed + 1) / 2);
  }
}
