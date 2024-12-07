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
public static double trackWidthMeters = 0.06;
  }
}
