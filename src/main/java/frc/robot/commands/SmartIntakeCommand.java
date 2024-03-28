package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends SequentialCommandGroup {
  public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, XboxController controller) {
    super(
        intakeSubsystem
            .run(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed))
            .until(intakeSubsystem::hasNote)
            .alongWith(intakeSubsystem.vibrateControllerOnNoteCommand(controller)));
  }
}
