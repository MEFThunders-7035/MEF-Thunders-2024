package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BasicRunShooterCommand extends SequentialCommandGroup {
  public BasicRunShooterCommand(ShooterSubsystem shooterSubsystem) {
    super(
        shooterSubsystem.run(
            () -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed)));
  }

  public BasicRunShooterCommand(ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem, double untilTimeSeconds) {
    super(
        new ParallelRaceGroup(
            new BasicRunShooterCommand(shooterSubsystem), // Run the shooter
            ledSubsystem.loadingAnimation(untilTimeSeconds)),
        shooterSubsystem.runOnce(
            () -> shooterSubsystem.setShooterSpeed(0) // Stop the shooter after the wait
            ));
  }
}
