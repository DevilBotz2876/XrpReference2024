package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystemXrp extends SubsystemBase implements Drive {
  private final DriveDifferentialIO io;
  private final DriveDifferentialIOInputsAutoLogged inputs =
      new DriveDifferentialIOInputsAutoLogged();

  private static final double wheelDiameterMeters = 0.060; // 60 mm
  private static final double trackWidthMeters = 0.155; // 155 mm

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(trackWidthMeters);

  // START: Setup Odometry
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private Field2d field = new Field2d();
  // END: Setup Odometry

  @AutoLogOutput private Pose2d pose;

  /** Creates a new DriveSubsystemXrp. */
  public DriveSubsystemXrp(DriveDifferentialIO io) {
    this.io = io;

    // START: Setup Odometry
    SmartDashboard.putData("Field", field);
    // END: Setup Odometry
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            Units.radiansToRotations(inputs.leftVelocityRadPerSec) * wheelDiameterMeters,
            Units.radiansToRotations(inputs.rightVelocityRadPerSec) * wheelDiameterMeters));
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    double leftSpeed;
    double rightSpeed;
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    leftSpeed = wheelSpeeds.leftMetersPerSecond / DriveConstants.maxLinearVelocityMetersPerSec;
    rightSpeed = wheelSpeeds.rightMetersPerSecond / DriveConstants.maxLinearVelocityMetersPerSec;

    io.tankDrive(leftSpeed, rightSpeed);
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.zRotation, new DifferentialDriveWheelPositions(0, 0), pose);
    io.resetEncoders();
    this.pose = pose;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // START: Setup Odometry
    pose =
        odometry.update(
            inputs.zRotation,
            new DifferentialDriveWheelPositions(
                Units.radiansToRotations(inputs.leftPositionRad) * wheelDiameterMeters,
                Units.radiansToRotations(inputs.rightPositionRad) * wheelDiameterMeters));
    field.setRobotPose(pose);
    // END: Setup Odometry
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}
    

