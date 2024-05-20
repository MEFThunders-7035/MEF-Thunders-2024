package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class MoveArmToPositionCommand extends ParallelRaceGroup {
  public MoveArmToPositionCommand(ArmSubsystem armSubsystem, double position) {
    this(armSubsystem, () -> position);
  }

  public MoveArmToPositionCommand(ArmSubsystem armSubsystem, DoubleSupplier position) {
    super(
        armSubsystem
            .run(() -> armSubsystem.setArmToPosition(position.getAsDouble()))
            .until(() -> armSubsystem.isArmAtPosition(position.getAsDouble())));
  }
}
