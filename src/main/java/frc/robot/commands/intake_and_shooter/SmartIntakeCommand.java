package frc.robot.commands.intake_and_shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends ParallelCommandGroup {
  public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, XboxController controller) {
    super(
        new BasicIntakeCommand(intakeSubsystem),
        intakeSubsystem.vibrateControllerOnNoteCommand(controller));
  }
}
