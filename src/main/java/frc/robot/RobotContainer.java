// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.TurnDegreesGyro;
import frc.robot.commands.auto.center.Center1Note;
import frc.robot.subsystems.arm.ArmIOXrp;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveDifferentialIOXrp;
import frc.robot.subsystems.drive.DriveSubsystemXrp;
import frc.robot.subsystems.intake.IntakeIOXrp;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOXrp;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.Map;

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
  // private final ArmSubsystem arm = new ArmSubsystem(new ArmIOStud());
  private final IntakeSubsystem intake = new IntakeSubsystem(new IntakeIOXrp(5));
  private final ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOXrp(3));

  private final CommandXboxController mainController = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser =  new SendableChooser<>();

  private boolean useYJoystick = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureShuffleboardCommands();

    Mechanism2d mech2d = new Mechanism2d(60, 60);
    arm.add2dSim(mech2d);
    SmartDashboard.putData("2D Mech", mech2d);

    autoChooser.setDefaultOption("None", new Command() {});
    autoChooser.addOption("Center 1 Note", new Center1Note(drive, arm, shooter, intake));
    autoChooser.addOption("1.5s Forward", new AutoDrive(drive));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        new ArcadeDrive(drive, () -> -mainController.getLeftY(), () -> -mainController.getLeftX()));

    // arm.setDefaultCommand(new ArmCommand(arm, () -> -mainController.getRightY()));

    EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    BooleanEvent armJoystickBooleanEvent =
        new BooleanEvent(eventLoop, () -> Math.abs(mainController.getRightY()) > 0.05)
            .and(() -> useYJoystick);

    Trigger armJoystickTrigger = armJoystickBooleanEvent.castTo(Trigger::new);
    armJoystickTrigger.whileTrue(
        new ArmCommand(arm, () -> MathUtil.applyDeadband(-mainController.getRightY(), 0.05)));

    BooleanEvent armUpPOVEvent =
        new BooleanEvent(eventLoop, () -> mainController.pov(0).getAsBoolean())
            .and(() -> !useYJoystick);
    BooleanEvent armDownPOVEvent =
        new BooleanEvent(eventLoop, () -> mainController.pov(180).getAsBoolean())
            .and(() -> !useYJoystick);

    Trigger armUpPOVTrigger = armUpPOVEvent.castTo(Trigger::new);
    armUpPOVTrigger.whileTrue(new SetArmAngleCommand(arm, () -> arm.getAngle() + 1));

    Trigger armDownPOVTrigger = armDownPOVEvent.castTo(Trigger::new);
    armDownPOVTrigger.whileTrue(new SetArmAngleCommand(arm, () -> arm.getAngle() - 1));

    intake.setDefaultCommand(new IntakeCommand(intake, () -> {
        if(mainController.leftBumper().getAsBoolean()) {
            return 1.0;
        }
        if(mainController.rightBumper().getAsBoolean()) {
            return -1.0;
        }
        return 0.0;}));

    shooter.setDefaultCommand(
        new ShooterCommand(
            shooter,
            () -> mainController.getRightTriggerAxis() * 0.85,
            () -> mainController.getLeftTriggerAxis() * 0.85));

    mainController
        .pov(270)
        .onTrue(
            new InstantCommand(
                () -> {
                  useYJoystick = !useYJoystick;
                }))
        .toggleOnTrue(
            new TankDrive(
                drive,
                () -> -mainController.getLeftY(),
                () -> -mainController.getRightY())); // .toggleOnFalse(new ArmCommand(arm, () ->
    // -mainController.getRightY()));
  }

  private void configureShuffleboardCommands() {
    ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands Tab");

    ShuffleboardLayout armLayout =
        commandsTab.getLayout("Arm", BuiltInLayouts.kGrid).withSize(2, 2);
    GenericEntry armAngle =
        armLayout
            .add("angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 180))
            .getEntry();
    armLayout.add("set angle", new SetArmAngleCommand(arm, () -> armAngle.getDouble(0.0)));

    ShuffleboardLayout driveLayout =
        commandsTab.getLayout("Drive", BuiltInLayouts.kGrid).withSize(2, 3);
    GenericEntry turnAngle =
        driveLayout
            .add("turn angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -359, "max", 359))
            .withPosition(0, 0)
            .getEntry();
    GenericEntry turnSpeed =
        driveLayout
            .add("turn speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1))
            .withPosition(0, 1)
            .getEntry();
    driveLayout
        .add(
            "turn angle command (gyro) (not working)",
            new TurnDegreesGyro(
                drive, () -> turnAngle.getDouble(0.0), () -> turnSpeed.getDouble(0.0)))
        .withPosition(0, 2);
    driveLayout
        .add(
            "turn angle command",
            new TurnDegrees(() -> turnSpeed.getDouble(0.0), () -> turnAngle.getDouble(0.0), drive))
        .withPosition(0, 2);

    driveLayout.add("drive", new AutoDrive(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.v
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
