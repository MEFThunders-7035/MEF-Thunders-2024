package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class SmartIntakeCommand extends ParallelCommandGroup {
  public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, XboxController controller) {
    super(
        new BasicIntakeCommand(intakeSubsystem, ledSubsystem),
        intakeSubsystem.vibrateControllerOnNoteCommand(controller));
  }
}
