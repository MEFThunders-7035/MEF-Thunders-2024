package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class BasicIntakeCommand extends SequentialCommandGroup {
  public BasicIntakeCommand(IntakeSubsystem intakeSubsystem) {
    super(
        intakeSubsystem
            .run(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed))
            .until(intakeSubsystem::hasNote));
  }
}
