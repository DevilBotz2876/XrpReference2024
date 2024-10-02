package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double speed;
  }

  /**
   * Updates the set of loggable inputs. Should be called periodically.
   *
   * @param inputs the set of loggable inputs
   */
  public void updateInputs(ShooterIOInputs inputs);

  // In the range [-1.0 .. 1.0], where -1.0 is max reverse speed and 1.0 max forward speed
  public void setSpeed(double speed);
}
