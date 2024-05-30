package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class SmartShootCommand extends ParallelRaceGroup {
  private static final double waitTime = 2.5;

  public SmartShootCommand(ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
    super(
        new SequentialCommandGroup(
            new BasicRunShooterCommand(shooterSubsystem, ledSubsystem, waitTime),
            Commands.race(intakeSubsystem.loadShooter(), new BasicRunShooterCommand(shooterSubsystem))));
  }

  public SmartShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      LEDSubsystem ledSubsystem,
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem) {
    super(
        // Constantly move the arm until all the other commands finish.
        armSubsystem.moveTo(driveSubsystem::getDistanceToShooter),
        new SmartShootCommand(shooterSubsystem, ledSubsystem, intakeSubsystem));
  }

  public SmartShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      LEDSubsystem ledSubsystem,
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed) {
    super(
        // Constantly move the arm until all the other commands finish.
        armSubsystem.moveTo(driveSubsystem::getDistanceToShooter),
        driveSubsystem.driveFacingShooter(xSpeed, ySpeed),
        // The finishing command will be this one:
        new SmartShootCommand(shooterSubsystem, ledSubsystem, intakeSubsystem));
  }
}
