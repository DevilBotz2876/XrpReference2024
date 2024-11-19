package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class TurnDegrees extends Command {
  private final Drive m_drive;
  private final Supplier<Double> m_degrees;
  private final Supplier<Double> m_speed;
  private double m_initialLeftDistance;
  private double m_initialRightDistance;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(Supplier<Double> speed, Supplier<Double> degrees, Drive drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements((Subsystem) drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialLeftDistance = m_drive.getLeftDistanceInch();
    m_initialRightDistance = m_drive.getRightDistanceInch();

    m_drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setChassisSpeeds(
        new ChassisSpeeds(0, 0, m_speed.get() * DriveConstants.maxAngularVelocityRadPerSec));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       XRP Chassis found here, https://www.sparkfun.com/products/22230,
       has a wheel placement diameter (163 mm) - width of the wheel (8 mm) = 155 mm
       or 6.102 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 6.102 / 360;
    double targetDistance = inchPerDegree * m_degrees.get();

    double leftDistanceTraveled = Math.abs(m_drive.getLeftDistanceInch() - m_initialLeftDistance);
    double rightDistanceTraveled =
        Math.abs(m_drive.getRightDistanceInch() - m_initialRightDistance);

    // Compare distance travelled from start to distance based on degree turn
    return (leftDistanceTraveled + rightDistanceTraveled) / 2.0 >= targetDistance;
  }
}
