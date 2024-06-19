package frc.robot.commands.intake_and_shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.led_commands.LEDBlinkRedCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSystem;

public class BasicIntakeCommand extends ParallelRaceGroup {
  public BasicIntakeCommand(IntakeSubsystem intakeSubsystem) {
    super(
        intakeSubsystem
            .runEnd(
                () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed),
                intakeSubsystem::stopMotors)
            .until(intakeSubsystem::hasNote),
        new LEDBlinkRedCommand(LEDSystem.getInstance()));
  }
}
