// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead,
 * the structure of
 * the robot (including subsystems, commands, and button mappings) should be
 * declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystemXrp drive = new DriveSubsystemXrp(new DriveDifferentialIOXrp());
  private final ArmSubsystem arm = new ArmSubsystem(new ArmIOXrp(4));
  private final IntakeSubsystem intake = new IntakeSubsystem(new IntakeIOXrp(5));
  private final ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOXrp(2));

  private final CommandXboxController mainController = new CommandXboxController(0);

  // Stores the number of times the "B" button has been pressed
  private int bPressCount = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Configure the robot to be driven by the left stick for steering and for
    // throttle
    drive.setDefaultCommand(
        new ArcadeDrive(drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getLeftX()));

    arm.setDefaultCommand(new ArmCommand(arm, () -> -mainController.getRightY()));

    intake.setDefaultCommand(new IntakeCommand(intake, () -> -mainController.getRightX()));

    shooter.setDefaultCommand(new ShooterCommand(shooter,
        () -> mainController.getLeftTriggerAxis(),
        () -> mainController.getRightTriggerAxis()));

    // toggles the drive mode between left stick controlling steering and right
    // stick controlling steering
    new Trigger(mainController.b().onTrue(new InstantCommand(this::handleBButtonPress)));
  }

  // toggles the drive mode between left stick controlling steering and right
  // stick controlling steering
  private void handleBButtonPress() {

    // Increment the button B press count
    bPressCount++;

    // Use modulo to cycle through control schemes
    int controlScheme = bPressCount % 3;

    /**
     * Switches between different control schemes for the robot's steering based on
     * the value of controlScheme.
     * Each case represents a different control scheme:
     * - Case 0: Left stick controls steering using ArcadeDrive.
     * - Case 1: Right stick controls steering using ArcadeDrive.
     * - Case 2: Both sticks control steering using TankDrive.
     *
     * @param controlScheme The current control scheme to use.
     */
    switch (controlScheme) {
      case 0:
        // Left stick controls steering
        new ArcadeDrive(drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getLeftX()).schedule();
        break;
      case 1:
        // Right stick controls steering
        new ArcadeDrive(
            drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getRightX()).schedule();
        break;
      case 2:
        // Both sticks control steering using TankDrive
        new TankDrive(
            drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getRightY()).schedule();
        break;
      default:
        // Default case (should not be reached)
        break;
    }
  };

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
