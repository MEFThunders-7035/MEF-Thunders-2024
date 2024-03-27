package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmIdleCommand extends SequentialCommandGroup {
  public ArmIdleCommand(ArmSubsystem armSubsystem, DoubleSupplier axis) {
    super(armSubsystem.run(() -> toRun(armSubsystem, axis.getAsDouble())));
  }

  private static void toRun(ArmSubsystem armSubsystem, double axis) {
    var data = SmartDashboard.getNumber("Arm Pos", 0);
    if (Math.abs(axis) < 0.02) {
      if (data != 0) {
        armSubsystem.setArmToPosition(data);
        return;
      }
      armSubsystem.setArmSpeed(0);
      return;
    }
    armSubsystem.setArmToPosition(axis);
  }
}
