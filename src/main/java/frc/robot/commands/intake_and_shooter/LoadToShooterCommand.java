package frc.robot.commands.intake_and_shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.util_commands.WaitANDConditionCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadToShooterCommand extends ParallelRaceGroup {
  public LoadToShooterCommand(IntakeSubsystem intakeSubsystem) {
    super(
        intakeSubsystem.runEnd(
            () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed, 0, true),
            intakeSubsystem::stopMotors),
        new WaitANDConditionCommand(0.5, () -> !intakeSubsystem.hasNote())
            .alongWith(new PrintCommand("Loading To Shooter"))
            .andThen(new PrintCommand("Loaded to Shooter!")));
  }
}
