package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.xrp.XRPMotor;

public class ShooterIOXrp implements ShooterIO {
  private final XRPMotor motor;
  private SlewRateLimiter filter = new SlewRateLimiter(4.0);

  public ShooterIOXrp(int deviceNum) {
    motor = new XRPMotor(deviceNum);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.speed = motor.get();
  }

  public void setSpeed(double speed) {
    speed = filter.calculate(speed);
    motor.set(speed);
  }
}
