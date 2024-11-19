package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

// This is currently not working well.
public class TurnDegreesGyro extends Command {
  private final Drive drive;
  private final Supplier<Double> speed;
  private final Supplier<Double> degrees;
  private final double degreesError;
  private double initialDegrees;
  private double goalDegrees;

  /**
   * The command rotates the robot a specified angle at a specified speed
   *
   * @param drive The drivetrain subsystem
   * @param speed The speed of rotation, between -1 and 1
   * @param degrees The desired degrees to turn
   */
  public TurnDegreesGyro(Drive drive, Supplier<Double> degrees, Supplier<Double> speed) {
    this.degrees = degrees;
    this.speed = () -> Math.max(-1.0, Math.min(speed.get(), 1.0));
    this.degreesError = 2;
    this.drive = drive;
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    initialDegrees = drive.getPose().getRotation().getDegrees();
    goalDegrees = (initialDegrees + degrees.get()) % 360;
    if (goalDegrees < 0) {
      goalDegrees += 360; // Ensure goalDegrees is positive
    }
  }

  @Override
  public void execute() {
    drive.tankDrive(
        -speed.get() * DriveConstants.maxLinearVelocityMetersPerSec,
        speed.get() * DriveConstants.maxLinearVelocityMetersPerSec);
  }

  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {

    double currentDegrees = drive.getPose().getRotation().getDegrees();
    double difference = Math.abs(goalDegrees - currentDegrees);

    // Ensures that difference is between 0-360
    if (difference > 180) {
      difference = 360 - difference;
    }

    return difference <= degreesError;
  }
}
