package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.ExtraFunctions;

public class MoveArmToShooterCommand extends SequentialCommandGroup {
  public MoveArmToShooterCommand(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem) {
    super(
        armSubsystem.run(
            () ->
                armSubsystem.setArmToPosition(
                    ExtraFunctions.getAngleFromDistance(driveSubsystem.getDistanceToShooter()))));
  }
}
