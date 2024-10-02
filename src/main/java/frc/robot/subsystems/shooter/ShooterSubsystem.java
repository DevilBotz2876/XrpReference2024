package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements Shooter {
  ShooterIO io;
  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public double getSpeed() {
    return inputs.speed;
  }
}
