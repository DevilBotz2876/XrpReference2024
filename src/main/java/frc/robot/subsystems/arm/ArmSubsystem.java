package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final double armAngle2dOffset = 0;
  private MechanismLigament2d arm2d = null;

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("arm", inputs);

    if (null != arm2d) {
      arm2d.setAngle(inputs.angleDeg + armAngle2dOffset);
    }
  }

  public void add2dSim(Mechanism2d mech2d) {
    MechanismRoot2d armPivot2d = mech2d.getRoot("Arm Pivot", 15, 10);
    armPivot2d.append(new MechanismLigament2d("Arm Tower", 10, -90));
    arm2d =
        armPivot2d.append(
            new MechanismLigament2d(
                "Arm", 30, inputs.angleDeg + armAngle2dOffset, 6, new Color8Bit(Color.kYellow)));
  }

  public void setAngle(double angleDeg) {
    io.setAngle(angleDeg);
  }

  public double getAngle() {
    return inputs.angleDeg;
  }
}
