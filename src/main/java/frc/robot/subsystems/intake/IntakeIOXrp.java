package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public class IntakeIOXrp implements IntakeIO {
  private final XRPServo motor;
  private SlewRateLimiter filter = new SlewRateLimiter(4.0);

  public IntakeIOXrp(int deviceNum) {
    motor = new XRPServo(deviceNum);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speed = motor.getPosition();
  }

  public void setSpeed(double speed) {
    speed = filter.calculate(speed);
    motor.setPosition(speed);
  }
}
