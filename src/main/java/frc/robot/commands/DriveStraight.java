package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class DriveStraight extends SequentialCommandGroup {
  private final Drive drive;
  private final double distance;      

  public DriveStraight(Drive drive, double distance) {
    this.drive = drive;
    this.distance = distance;
    
    double divisionFactor = 0.10;
    double distanceDivided = distance * divisionFactor;

    while (distanceDivided > 1) {
      divisionFactor -= 0.01;
      distanceDivided = distance * divisionFactor;
    }

    double crazyTimes = (100 / divisionFactor) / 10;

    for (double x = 0; x < (crazyTimes * 0.050); x++) {
      addCommands(
        new DriveTurn(drive, 0.2),
        new DriveTurn(drive, -0.32)
      );
    }
  }
}
