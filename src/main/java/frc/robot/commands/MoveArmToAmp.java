package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToAmp extends SequentialCommandGroup {
  MoveArmToAmp(ArmSubsystem armSubsystem) {
    super(
        armSubsystem
            .run(() -> armSubsystem.setArmToPosition(0.5))
            .until(() -> armSubsystem.isArmAtPosition(0.5)));
  }
}
