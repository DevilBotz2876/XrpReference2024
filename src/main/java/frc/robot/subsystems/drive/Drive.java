package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drive {
  public ChassisSpeeds getChassisSpeeds();

  public void setChassisSpeeds(ChassisSpeeds speeds);

  public Pose2d getPose();

  public void setPose(Pose2d pose);

}
