package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadToShooterCommand extends ParallelRaceGroup {
  public LoadToShooterCommand(IntakeSubsystem intakeSubsystem) {
    super(
        intakeSubsystem.run(
            () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed, 0, true)),
        new UntilWithTimeCommand(0.5, () -> !intakeSubsystem.hasNote())
            .alongWith(new PrintCommand("Loading To Shooter"))
            .andThen(new PrintCommand("Loaded to Shooter!")));
  }
}
