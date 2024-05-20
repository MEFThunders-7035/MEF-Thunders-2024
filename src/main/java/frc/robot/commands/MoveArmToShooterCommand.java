package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.ExtraFunctions;

public class MoveArmToShooterCommand extends RunCommand {
  /** This Command Never Ends! */
  public MoveArmToShooterCommand(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem) {
    super(
        () ->
            armSubsystem.setArmToPosition(
                ExtraFunctions.getAngleFromDistance(driveSubsystem.getDistanceToShooter())),
        armSubsystem);
  }
}
