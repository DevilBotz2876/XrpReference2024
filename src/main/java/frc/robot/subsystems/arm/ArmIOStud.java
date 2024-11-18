package frc.robot.subsystems.arm;

public class ArmIOStud implements ArmIO {

  private double angle;

  public ArmIOStud() {}

  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleDeg = angle;
  }

  public void setAngle(double angleDeg) {
    angle = angleDeg;
  }
}
