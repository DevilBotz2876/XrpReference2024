package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;

/**
 * The DriveDifferentialIOXrp class implements the DriveDifferentialIO interface
 * for an XRP robot. This class provides the necessary functionality to control
 * the differential drive system, including motor control, encoder readings, 
 * and gyro integration.
 */
public class DriveDifferentialIOXrp implements DriveDifferentialIO {
    /**
   * The gear ratio of the drive system, calculated as the product of the 
   * individual gear ratios in the gear train. The gear ratio is 48.75:1.
   * 
   * Calculation:
   * (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0) = 48.75
   */
  private static final double gearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1

  /**
   * The number of encoder counts per revolution of the motor shaft.
   */
  private static final double countsPerMotorShaftRev = 12.0;

  /**
   * The number of encoder counts per revolution of the output shaft, taking 
   * nto account the gear ratio.
   * 
   * Calculation:
   * countsPerMotorShaftRev * gearRatio = 12.0 * 48.75 = 585.0 nm
   */
  private static final double countsPerRevolution = countsPerMotorShaftRev * gearRatio; // 585.0

  // The XRP has the left and right motors set to channels 0 and 1 respectively
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded to use DIO pins 4/5 
  // and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

   /**
   * Constructs a new DriveDifferentialIOXrp instance.
   * Initializes the encoders to use meters as the unit for distances and resets the encoders.
   * Inverts the right motor since it is flipped.
   */
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
    // Update the left and right wheel velocities in radians per second
    inputs.leftVelocityRadPerSec = leftEncoder.getRate();
    inputs.rightVelocityRadPerSec = rightEncoder.getRate();

    // Update the left and right wheel positions in radians
    inputs.leftPositionRad = leftEncoder.getDistance();
    inputs.rightPositionRad = rightEncoder.getDistance();

    // Update the robot's orientation using the gyro
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
