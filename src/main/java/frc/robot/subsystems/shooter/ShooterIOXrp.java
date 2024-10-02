package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.xrp.XRPMotor;

public class ShooterIOXrp implements ShooterIO {
  private final XRPMotor motor;

  public ShooterIOXrp(int deviceNum) {
    motor = new XRPMotor(deviceNum);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.speed = motor.get();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
