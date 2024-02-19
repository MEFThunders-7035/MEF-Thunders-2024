package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SmartShootCommand extends SequentialCommandGroup {
  public SmartShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    super(
        new RunCommand(
                () -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed),
                shooterSubsystem)
            .withTimeout(1)
            .andThen(
                // This command is required, so the default command doesn't override it to 0.
                new RunCommand(
                        () -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed),
                        shooterSubsystem)
                    .deadlineWith(intakeSubsystem.loadToShooterCommand())));
  }
}
