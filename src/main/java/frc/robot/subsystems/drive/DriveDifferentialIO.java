package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The DriveDifferentialIO interface defines the methods and inner classes
 * required for
 * controlling a differential drive system. Implementations of this interface
 * should
 * provide the necessary functionality to update inputs, control the drive
 * system,
 * and reset encoders.
 *
 * An interface in Java is a reference type, similar to a class, that can
 * contain only constants, method signatures, default methods, static methods,
 * and nested types. Interfaces cannot contain instance fields or constructors.
 * They are used to specify a set of methods that a class must implement.
 */
public interface DriveDifferentialIO {
  /**
   * The DriveDifferentialIOInputs class holds the loggable inputs for the
   * differential drive system.
   * This class is annotated with @AutoLog to indicate that its fields should be
   * automatically logged.
   */
  @AutoLog
  public static class DriveDifferentialIOInputs {
    public double leftVelocityRadPerSec;
    public double rightVelocityRadPerSec;
    public double leftPositionRad;
    public double rightPositionRad;

    public Rotation2d xRotation;
    public Rotation2d yRotation;
    public Rotation2d zRotation;
  }

  /**
   * Updates the set of loggable inputs. Should be called periodically.
   *
   * @param inputs the set of loggable inputs
   */
  public void updateInputs(DriveDifferentialIOInputs inputs);

  /**
   * Controls the differential drive system using tank drive.
   *
   * @param leftSpeed  The speed of the left side of the drive system.
   * @param rightSpeed The speed of the right side of the drive system.
   */
  public void tankDrive(double leftSpeed, double rightSpeed);

  /**
   * Resets the encoders of the differential drive system.
   */
  public void resetEncoders();
}
