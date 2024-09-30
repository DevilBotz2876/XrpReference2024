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
    // In general, the 0-180 servos that are supported by XRP use a PWM range of 1000us to 2000us.
    // The continuous servos that we bought from Amazon
    // (https://www.amazon.com/gp/product/B092VN3MTX)
    //   have a 900 us to 2100 us range, where 900 is max CCW, 2100 us max CW, and 1500 is stopped.
    //
    // Pulse Width (us) | 1000     | 1500    |  2000   |
    //      0-180 servo | 0 deg    | 45deg   |  90deg  |
    // continuous servo | ~max CCW | stopped | ~max CW |
    //
    // Since the servos operate on the same overlapping range, for the continuous servo,
    // we need to scale the requested speed [-1.0 .. 1.0] to a servo "position" [0.0 .. 1.0], where
    // "0.5" is stopped for a continuous servo. This will map to the correct PWM signal to the
    // servo.
    //
    // Note: since the XRP servo API operates in the 1000-2000 us range, we actually cannot
    // achieve the absolute max speed of the continuous servo.
    //
    servo.setPosition((speed + 1) / 2);
  }
}
