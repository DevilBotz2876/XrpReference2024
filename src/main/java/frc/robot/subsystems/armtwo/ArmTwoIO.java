package frc.robot.subsystems.armtwo;

import org.littletonrobotics.junction.AutoLog;

public interface ArmTwoIO {
  @AutoLog
  public static class ArmIOInputs {
    public double angleDeg;
  }

  /**
   * Updates the set of loggable inputs. Should be called periodically.
   *
   * @param inputs the set of loggable inputs
   */
  public void updateInputs(ArmIOInputs inputs);

  public void setAngle(double angleDeg);
}
