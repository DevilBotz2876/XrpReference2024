// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.arm.ArmIOXrp;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveDifferentialIOXrp;
import frc.robot.subsystems.drive.DriveSubsystemXrp;
import frc.robot.subsystems.intake.IntakeIOXrp;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOXrp;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystemXrp drive = new DriveSubsystemXrp(new DriveDifferentialIOXrp());
  private final ArmSubsystem arm = new ArmSubsystem(new ArmIOXrp(4));
  private final IntakeSubsystem intake = new IntakeSubsystem(new IntakeIOXrp(5));
  private final ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOXrp(2));

  private final CommandXboxController mainController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drive.setDefaultCommand(
    //     new ArcadeDrive(drive, 
    //           () -> -mainController.getLeftY(), 
    //           () -> -mainController.getLeftX()));

    // Arcade drive but left stick for throttle and right stick controls steering
    // drive.setDefaultCommand(
    //     new ArcadeDrive(drive,
    //         () -> -mainController.getLeftY(),
    //         () -> -mainController.getRightX()));

    // Both sticks control steering using TankDrive
    // Note when this is enabled intake should be disabled or mapped to different
    // logic as it uses getRightX for control 
    drive.setDefaultCommand(
           new TankDrive(drive,
              () -> -mainController.getLeftY(),
              () -> -mainController.getRightY()));


    arm.setDefaultCommand(new ArmCommand(arm, () -> -mainController.getRightY()));
    // intake.setDefaultCommand(new IntakeCommand(intake, () -> -mainController.getRightX()));
    shooter.setDefaultCommand(
        new ShooterCommand(
            shooter,
            () -> mainController.getLeftTriggerAxis(),
            () -> mainController.getRightTriggerAxis()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
