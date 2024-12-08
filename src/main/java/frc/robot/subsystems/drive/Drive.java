package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drive {
  /**
   * Gets the current chassis speeds.
   *
   * @return The current chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds();

  /**
   * Sets the chassis speeds.
   *
   * @param speeds The chassis speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds);

  /**
   * Gets the current pose of the robot.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose();

  /**
   * Sets the pose of the robot. The pose includes the position (x, y coordinates) and orientation
   * (heading) of the robot. This method is typically used to reset the robot's position and
   * orientation during initialization or after a significant event, such as a collision or manual
   * repositioning.
   *
   * @param pose The pose to set, which includes the x and y coordinates and the orientation.
   */
  public void setPose(Pose2d pose);
}
