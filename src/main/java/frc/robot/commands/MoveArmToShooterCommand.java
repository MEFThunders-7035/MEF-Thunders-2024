package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.ExtraFunctions;

public class MoveArmToShooterCommand extends MoveArmToPositionCommand {
  public MoveArmToShooterCommand(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem) {
    super(
        armSubsystem,
        () -> ExtraFunctions.getAngleFromDistance(driveSubsystem.getDistanceToShooter()));
  }
}
