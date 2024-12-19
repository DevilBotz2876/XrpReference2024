package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The Drive interface defines the methods required for controlling the robot's
 * drive system.  Implementations of this interface should provide the necessary
 * functionality to get and set the chassis speeds and the robot's pose.
 * 
 * An interface in Java is a reference type, similar to a class, that can
 * contain only constants, method signatures, default methods, static methods,
 * and nested types. Interfaces cannot contain instance fields or constructors.
 * They are used to specify a set of methods that a class must implement.
 */
public interface Drive {
  public ChassisSpeeds getChassisSpeeds();

  public void setChassisSpeeds(ChassisSpeeds speeds);

  public Pose2d getPose();

  public void setPose(Pose2d pose);
}
