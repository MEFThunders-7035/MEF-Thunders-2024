package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootToAmpCommand extends SequentialCommandGroup {
  public ShootToAmpCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem) {
    super(
        new MoveArmToAmp(armSubsystem).raceWith(new BasicRunShooterCommand(shooterSubsystem)),
        intakeSubsystem
            .loadToShooterCommand()
            .raceWith(new BasicRunShooterCommand(shooterSubsystem)));
  }
}
