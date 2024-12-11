// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class DriveConstants {
    // The maximum linear velocity of the robot in meters per second.
    // This represents the highest speed at which the robot can move forward or backward.
    public static double maxLinearVelocityMetersPerSec = 1.0;

    // The maximum angular velocity of the robot in radians per second.
    // This represents the highest speed at which the robot can rotate in place.
    public static double maxAngularVelocityRadPerSec = 4 * 2 * Math.PI;

    // The track width of the robot in meters.
    // This is the distance between the left and right wheels of the robot.
    public static double trackWidthMeters = 0.155;

    // The diameter of the robot's wheels in meters (60 mm).
    public static double wheelDiameterMeters = 0.060;

    public class speedPIDConstants {
      // The proportional gain for the PID controller.
      // This value determines how strongly the controller reacts to the current error.
      // A higher kP value will result in a stronger response to the error, but may cause overshooting.
      public static double kP = .5;

      // The integral gain for the PID controller.
      // This value determines how strongly the controller reacts to the accumulation of past errors.
      // A higher kI value will result in a stronger response to accumulated errors, but may cause instability.
      public static double kI = 0.00;

      // The derivative gain for the PID controller.
      // This value determines how strongly the controller reacts to the rate of change of the error.
      // A higher kD value will result in a stronger response to the rate of change, helping to dampen oscillations.
      public static double kD = .5;
    }

    public class zRotationPIDConstants {
      // The proportional gain for the PID controller.
      // This value determines how strongly the controller reacts to the current error.
      // A higher kP value will result in a stronger response to the error, but may cause overshooting.
      public static double kP = 0.01;

      // The integral gain for the PID controller.
      // This value determines how strongly the controller reacts to the accumulation of past errors.
      // A higher kI value will result in a stronger response to accumulated errors, but may cause instability.
      public static double kI = 0.0;

      // The derivative gain for the PID controller.
      // This value determines how strongly the controller reacts to the rate of change of the error.
      // A higher kD value will result in a stronger response to the rate of change, helping to dampen oscillations.
      public static double kD = 0.01;
    }


  }
}
