package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReallySimpleAuto extends SequentialCommandGroup {
  public ReallySimpleAuto(
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      DriveSubsystem driveSubsystem) {
    super(
        new SmartShootCommand(shooterSubsystem, intakeSubsystem, armSubsystem, driveSubsystem),
        new WaitCommand(1)
            .andThen(
                driveSubsystem
                    .run(() -> driveSubsystem.drive(-0.25, 0, 0))
                    .raceWith(new WaitCommand(2.5))));
  }
}
