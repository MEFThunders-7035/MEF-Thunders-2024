package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class SmartShootCommand extends ParallelRaceGroup {
  private static final double waitTime = 2.5;

  public SmartShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    super(
        new BasicRunShooterCommand(shooterSubsystem, waitTime)
            .andThen(
                // This command is required, so the default command doesn't override it to 0.
                new BasicRunShooterCommand(shooterSubsystem)
                    .raceWith(intakeSubsystem.loadToShooterCommand())));
  }

  public SmartShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem) {
    super(
        // Constantly move the arm until all the other commands finish.
        new MoveArmToShooterCommand(armSubsystem, driveSubsystem)
            .raceWith(
                new BasicRunShooterCommand(shooterSubsystem, waitTime)
                    .andThen(
                        new BasicRunShooterCommand(shooterSubsystem)
                            .raceWith(intakeSubsystem.loadToShooterCommand()))));
  }

  public SmartShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed) {
    super(
        // Constantly move the arm until all the other commands finish.
        new MoveArmToShooterCommand(armSubsystem, driveSubsystem),
        new DriveFacingShooter(driveSubsystem, xSpeed, ySpeed),
        // The finishing command will be this one:
        new BasicRunShooterCommand(shooterSubsystem, waitTime)
            .andThen(
                new BasicRunShooterCommand(shooterSubsystem)
                    .raceWith(intakeSubsystem.loadToShooterCommand())));
  }
}
