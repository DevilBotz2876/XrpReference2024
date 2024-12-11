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
    // Since the XRP standard 0-180 and the referenced continuous servos operate over roughly the
    // same overlapping range, for the continuous servo, we need to scale the requested speed
    // [-1.0 .. 1.0] to either a servo "position" [0.0 .. 1.0] or servo "angle" [0 .. 180]
    // where "0.5" or "180 degrees" is "stopped" for the continuous servo.
    //
    // Pulse Width (us) | 1000     | 1500    | 2000    |
    //            angle | 0 deg    | 90deg   | 180deg  |
    //         position | 0        | 0.5     | 1.0     |
    // continuous servo | ~max CCW | stopped | ~max CW |
    //
    // Note: since the XRP servo API operates in the 1000-2000us range, we actually cannot achieve
    // the absolute max speed of the continuous servo (900 or 2100us).
    //
    servo.setPosition((speed + 1) / 3);
  }
}
