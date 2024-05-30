package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends SequentialCommandGroup {
  public DefaultDriveCommand(DriveSubsystem driveSubsystem, XboxController controller) {
    super(
        driveSubsystem.run(
            () ->
                driveSubsystem.driveWithExtras(
                    controller.getLeftY(),
                    -controller.getLeftX(), // ! THIS IS INVERTED BECAUSE +y IS LEFT of robot
                    -controller.getRightX(), // ! THIS IS INVERTED BECAUSE + IS COUNTERCLOCKWISE
                    controller.getRightTriggerAxis())));
  }
}
