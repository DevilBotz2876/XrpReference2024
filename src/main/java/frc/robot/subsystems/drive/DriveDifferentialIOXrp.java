package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;

public class DriveDifferentialIOXrp implements DriveDifferentialIO {

  /**
   * The gear ratio of the drive system.
   * This is calculated as the product of the ratios of each stage in the gear
   * train.
   * The gear ratio determines how many times the motor shaft must rotate to
   * achieve one full rotation of the output shaft.
   * In this case, the gear ratio is calculated as:
   * (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0) = 48.75
   * This means the motor shaft rotates 48.75 times for each rotation of the
   * output shaft.
   */
  private static final double gearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1

  /**
   * The number of encoder counts per revolution of the motor shaft.
   * This value is specific to the encoder being used and represents how many
   * pulses the encoder generates for one full rotation of the motor shaft.
   * In this case, the encoder generates 12 counts per revolution of the motor
   * shaft.
   */
  private static final double countsPerMotorShaftRev = 12.0;

  /**
   * The number of encoder counts per revolution of the output shaft.
   * This is calculated by multiplying the counts per motor shaft revolution by
   * the gear ratio.
   * It represents how many pulses the encoder generates for one full rotation of
   * the output shaft.
   * For this drive system, it is calculated as:
   * countsPerMotorShaftRev * gearRatio = 12.0 * 48.75 = 585.0
   * This means the encoder generates 585 counts for each full rotation of the
   * output shaft.
   */
  private static final double countsPerRevolution = countsPerMotorShaftRev * gearRatio; // 585.0

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro gyro = new XRPGyro();

  public DriveDifferentialIOXrp() {
    // Set the distance per pulse for the left and right encoders using meters as
    // the unit. The distance per pulse is calculated as the circumference of the
    // wheel divided by the number of encoder counts per revolution.
    // The circumference of the wheel is given by the formula 2 * π * radius.
    // Assuming the radius of the wheel is 1 meter, the distance per pulse is
    // calculated as (2 * Math.PI) / countsPerRevolution.
    // This means each pulse from the encoder corresponds to a specific distance
    // traveled by the wheel, allowing for accurate measurement of the robot's
    // movement.
    leftEncoder.setDistancePerPulse((Math.PI * 2) / countsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * 2) / countsPerRevolution);

    /**
     * Resets the encoders to their initial state.
     * This method is typically called during the initialization of the drive system
     * to ensure that the encoder readings start from zero.
     * By resetting the encoders, any previous counts or distances recorded by the
     * encoders are cleared.
     * This is important for accurate tracking of the robot's movement, as it
     * ensures that the encoder values reflect the current position and movement of
     * the robot from a known starting point.
     */
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
