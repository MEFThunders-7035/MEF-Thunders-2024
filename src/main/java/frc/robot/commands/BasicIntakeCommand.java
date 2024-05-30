package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class BasicIntakeCommand extends ParallelRaceGroup {
  public BasicIntakeCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    super(
        intakeSubsystem
            .run(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed))
            .until(intakeSubsystem::hasNote),
        ledSubsystem.getRedBlinkCommand());
  }
}
