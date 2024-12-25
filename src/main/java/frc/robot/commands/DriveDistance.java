// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveDistance extends Command {
    private final Drive drive;
    private final PIDController distancePidController;
    private final PIDController anglePidController;
    private final Pose2d initialPose;
    private final Pose2d targetPose;
    private final double speed;

    /**
     * Creates a new DriveDistance. This command will drive your robot for a
     * desired distance at a desired speed.
     *
     * @param speed  The speed at which the robot will drive
     * @param meters The number of meters the robot will drive
     * @param drive  The drivetrain subsystem on which this command will run
     */
    public DriveDistance(
            double driveSpeed,
            double meters,
            Drive driveSubsystem,
            PIDController anglePidController,
            PIDController distancePidController) {

        this.speed = driveSpeed;
        this.drive = driveSubsystem;
        this.distancePidController = distancePidController;
        this.anglePidController = anglePidController;
        this.initialPose = drive.getPose();
        this.targetPose = new Pose2d(
                initialPose.getTranslation().plus(new Translation2d(meters, 0)),
                initialPose.getRotation());

        addRequirements((Subsystem) drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        distancePidController.reset();
        anglePidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
     //   double distanceError = targetPose.getTranslation().getX() - currentPose.getTranslation().getX();
        double angleError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

      // double distanceOutput = distancePidController.calculate(distanceError, 0);
        //double angleOutput = anglePidController.calculate(angleError, 0);
        double angleOutput = anglePidController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        drive.setChassisSpeeds(new ChassisSpeeds(speed, 0, angleOutput));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(targetPose.getTranslation().getX() - drive.getPose().getTranslation().getX()) < 0.01;
    }
}