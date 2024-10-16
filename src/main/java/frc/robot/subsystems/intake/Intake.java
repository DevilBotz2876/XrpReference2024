package frc.robot.subsystems.intake;

public interface Intake {
  // In the range [-1.0 .. 1.0], where -1.0 is max reverse speed and 1.0 max forward speed
  public void setSpeed(double speed);

  public double getSpeed();
}
