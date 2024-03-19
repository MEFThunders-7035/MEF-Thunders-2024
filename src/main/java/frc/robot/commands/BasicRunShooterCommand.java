package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class BasicRunShooterCommand extends SequentialCommandGroup {
  BasicRunShooterCommand(ShooterSubsystem shooterSubsystem) {
    super(
        shooterSubsystem.run(
            () -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed)));
  }

  BasicRunShooterCommand(ShooterSubsystem shooterSubsystem, double untilTimeSeconds) {
    super(
        shooterSubsystem
            .run(() -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed))
            .raceWith(new WaitCommand(untilTimeSeconds)));
  }
}
