package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends SequentialCommandGroup {
  public SmartIntakeCommand(
      IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, XboxController controller) {
    super(
        armSubsystem
            .setArmToPositionCommand(0)
            .onlyIf(() -> !intakeSubsystem.hasNote())
            .beforeStarting(new PrintCommand("Setting Arm to 0"))
            .andThen(
                intakeSubsystem.run(
                    () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed)))
            .alongWith(intakeSubsystem.vibrateControllerOnNoteCommand(controller)));
  }
}
