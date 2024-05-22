package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.led_commands.LEDLoadingWaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class BasicRunShooterCommand extends ParallelRaceGroup {
  public BasicRunShooterCommand(ShooterSubsystem shooterSubsystem) {
    super(
        shooterSubsystem.runEnd(
            () -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed),
            shooterSubsystem::stopShooter));
  }

  public BasicRunShooterCommand(ShooterSubsystem shooterSubsystem, double untilTimeSeconds) {
    super(
        new BasicRunShooterCommand(shooterSubsystem), // Run the shooter
        new LEDLoadingWaitCommand(untilTimeSeconds));
  }
}
