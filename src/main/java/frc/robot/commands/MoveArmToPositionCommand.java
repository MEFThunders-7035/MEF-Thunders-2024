package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class MoveArmToPositionCommand extends ParallelRaceGroup {
  public MoveArmToPositionCommand(ArmSubsystem armSubsystem, double position) {
    super(
        armSubsystem.run(() -> armSubsystem.setArmToPosition(position)),
        new WaitUntilCommand(() -> armSubsystem.isArmAtPosition(position)));
  }

  public MoveArmToPositionCommand(ArmSubsystem armSubsystem, DoubleSupplier position) {
    super(
        armSubsystem.run(() -> armSubsystem.setArmToPosition(position.getAsDouble())),
        new WaitUntilCommand(() -> armSubsystem.isArmAtPosition(position.getAsDouble())));
  }

  private static double clampPos(double pos) {
    return MathUtil.clamp(pos, 0, 0.5);
  }
}
