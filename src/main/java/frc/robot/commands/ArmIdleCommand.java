package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmIdleCommand extends SequentialCommandGroup {
  public ArmIdleCommand(ArmSubsystem armSubsystem, DoubleSupplier axis) {
    super(armSubsystem.run(() -> toRun(armSubsystem, axis.getAsDouble())));
    SmartDashboard.putNumber("Arm Pos", 0);
  }

  private static double getArmSetpoint(ArmSubsystem armSubsystem, double axis) {
    var data = SmartDashboard.getNumber("Arm Pos", 0);
    if (Math.abs(axis) > 0.02) {
      return axis;
    }

    if (data != 0) {
      return data;
    }

    return 0;
  }

  private static void toRun(ArmSubsystem armSubsystem, double axis) {
    var data = SmartDashboard.getNumber("Arm Pos", 0);
    if (Math.abs(axis) > 0.02) {
      armSubsystem.setArmToPosition(axis);
      return;
    }

    if (data != 0) {
      armSubsystem.setArmToPosition(data);
      return;
    }

    armSubsystem.setArmToPosition(0);
  }
}
