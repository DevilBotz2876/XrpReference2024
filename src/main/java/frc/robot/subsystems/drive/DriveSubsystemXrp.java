package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystemXrp extends SubsystemBase implements Drive {
  // The IO subsystem used to interface with the robot's differential drive
  // hardware.
  private final DriveDifferentialIO io;

  // The inputs for the differential drive, automatically logged.
  private final DriveDifferentialIOInputsAutoLogged inputs =
      new DriveDifferentialIOInputsAutoLogged();

  // The diameter of the robot's wheels in meters (60 mm).
  private static final double wheelDiameterMeters = 0.060;

  // The track width of the robot in meters (155 mm).
  private static final double trackWidthMeters = 0.155;

  // The kinematics object used to convert between chassis speeds and wheel
  // speeds.
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(trackWidthMeters);

  // The odometry object used to track the robot's position on the field.
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

  // The field object used to represent the robot's position on the dashboard
  // view.
  private Field2d field = new Field2d();

  /**
   * The current pose of the robot, including its position and orientation. This field is
   * automatically logged.
   */
  @AutoLogOutput private Pose2d pose;

  /**
   * Constructs a new DriveSubsystemXrp.
   *
   * @param io The IO subsystem used to interface with the robot's differential drive hardware.
   */
  public DriveSubsystemXrp(DriveDifferentialIO io) {
    this.io = io;
  }

  /**
   * Gets the current chassis speeds of the robot.
   *
   * @return The current chassis speeds, represented as a ChassisSpeeds object. This includes the
   *     linear and angular velocities of the robot.
   */
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            Units.radiansToRotations(inputs.leftVelocityRadPerSec) * wheelDiameterMeters,
            Units.radiansToRotations(inputs.rightVelocityRadPerSec) * wheelDiameterMeters));
  }

  /**
   * Sets the desired chassis speeds for the robot. This method converts the desired chassis speeds
   * (linear and angular velocities) into individual wheel speeds and then drives the robot using
   * those wheel speeds.
   *
   * @param speeds The desired chassis speeds, represented as a {@link ChassisSpeeds} object. This
   *     includes the linear and angular velocities of the robot.
   */
  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    // Normalize wheel speeds by the maximum linear velocity (highest speed at which
    // the robot can move in a straight line)
    double leftSpeed =
        wheelSpeeds.leftMetersPerSecond / DriveConstants.maxLinearVelocityMetersPerSec;
    double rightSpeed =
        wheelSpeeds.rightMetersPerSecond / DriveConstants.maxLinearVelocityMetersPerSec;

    // Drive the robot using the calculated wheel speeds
    io.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Gets the current pose of the robot. The pose represents the robot's position and orientation on
   * the field. It is typically used for tracking the robot's location during autonomous and
   * teleoperated periods.
   *
   * @return The current pose of the robot as a {@link Pose2d} object.
   */
  public Pose2d getPose() {
    return pose;
  }

  /**
   * Sets a new pose for the robot and resets the odometry and encoders. This method is typically
   * used to reset the robot's position and orientation on the field, such as at the start of a
   * match or after a significant event that requires reinitialization.
   *
   * @param pose The new pose to set for the robot, represented as a {@link Pose2d} object. This
   *     includes the x and y coordinates and the orientation (heading) of the robot.
   */
  public void setPose(Pose2d pose) {
    // Reset the odometry to the new pose with zeroed wheel positions
    odometry.resetPosition(inputs.zRotation, new DifferentialDriveWheelPositions(0, 0), pose);

    // Reset the encoders to ensure accurate tracking from the new pose
    io.resetEncoders();

    // Update the internal pose state to the new pose
    this.pose = pose;
  }

  /**
   * This method is called periodically by the scheduler. It updates the robot's inputs, processes
   * those inputs, updates the robot's pose using odometry, and updates the field with the robot's
   * current pose. This ensures that the robot's state is consistently updated and accurately
   * reflects its position and orientation on the field.
   */
  @Override
  public void periodic() {
    // Update inputs from the IO subsystem
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update the robot's pose using odometry
    pose =
        odometry.update(
            inputs.zRotation,
            new DifferentialDriveWheelPositions(
                Units.radiansToRotations(inputs.leftPositionRad) * wheelDiameterMeters,
                Units.radiansToRotations(inputs.rightPositionRad) * wheelDiameterMeters));

    // Update the field with the robot's current pose
    field.setRobotPose(pose);
  }

  /**
   * The current position of the right wheels in meters.
   *
   * @return the current position of the right wheels (in meters)
   */
  public double getRightPositionMeters() {
    return Units.radiansToRotations(inputs.rightPositionRad) * wheelDiameterMeters;
  }

  /**
   * The current position of the right wheels in meters.
   *
   * @return the current position of the right wheels (in meters)
   */
  public double getLeftPositionMeters() {
    return Units.radiansToRotations(inputs.leftPositionRad) * wheelDiameterMeters;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
