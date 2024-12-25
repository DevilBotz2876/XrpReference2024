package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {
    private final Drive drive;
    private final PIDController distancePidController;
    private final PIDController anglePidController;
    private final Pose2d targetPose;
    private final Pose2d startPose;
    private final double speed;
    
    public DriveToPose(
            double driveSpeed,
            Pose2d targetPose,
            Drive driveSubsystem,
            PIDController distancePidController,
            PIDController anglePidController) {

        this.speed = driveSpeed;
        this.drive = driveSubsystem;
        this.distancePidController = distancePidController;
        this.anglePidController = anglePidController;
        
        this.startPose = drive.getPose();
        this.targetPose = targetPose;
        
        addRequirements((Subsystem) drive);   
    }

    @Override
    public void initialize() {
        drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        distancePidController.reset();
        anglePidController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
  
        double distanceOutput = distancePidController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double angleOutput = anglePidController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        angleOutput = 0;
        drive.setChassisSpeeds(new ChassisSpeeds(distanceOutput * speed, 0, angleOutput));
    }

    @Override
    public void end(boolean interrupted) {
        drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drive.getPose();
        boolean distanceFinished = Math.abs(targetPose.getTranslation().getX() - currentPose.getTranslation().getX()) < 0.01;
        boolean angleFinished = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) < 0.01;

        return distanceFinished && angleFinished;
    }
}