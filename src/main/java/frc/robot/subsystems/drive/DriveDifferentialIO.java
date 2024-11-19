package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DriveDifferentialIO {
  @AutoLog
  public static class DriveDifferentialIOInputs {
    public double leftVelocityRadPerSec;
    public double rightVelocityRadPerSec;
    public double leftPositionRad;
    public double rightPositionRad;
    public double leftPositionInch;
    public double rightPositionInch;

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

  public void tankDrive(double leftSpeed, double rightSpeed);

  public void resetEncoders();
}
