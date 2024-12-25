package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import frc.robot.util.XRPEncoder;

public class DriveDifferentialIOXrp implements DriveDifferentialIO {
  private static final double gearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double countsPerMotorShaftRev = 12.0;
  private static final double countsPerRevolution = countsPerMotorShaftRev * gearRatio; // 585.0

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new XRPEncoder(4, 5);
  private final Encoder rightEncoder = new XRPEncoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive differentialDrive =
      new DifferentialDrive(leftMotor::set, rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro gyro = new XRPGyro();

  public DriveDifferentialIOXrp() {
    // Use meters as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * 2) / countsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * 2) / countsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);
  }

  @Override
  public void updateInputs(DriveDifferentialIOInputs inputs) {
    inputs.leftVelocityRadPerSec = leftEncoder.getRate();
    inputs.rightVelocityRadPerSec = rightEncoder.getRate();
    inputs.leftPositionRad = leftEncoder.getDistance();
    inputs.rightPositionRad = rightEncoder.getDistance();

    inputs.xRotation = Rotation2d.fromDegrees(gyro.getAngleX());
    inputs.yRotation = Rotation2d.fromDegrees(gyro.getAngleY());
    inputs.zRotation = Rotation2d.fromDegrees(gyro.getAngleZ());
  }

  @Override
  public void tankDrive(double leftVelocity, double rightVelocity) {
    differentialDrive.tankDrive(leftVelocity, rightVelocity);
  }

  @Override
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
}
