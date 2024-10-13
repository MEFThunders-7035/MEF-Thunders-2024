package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends SequentialCommandGroup {
  // This is the minimum value the robot will slow down to when the left trigger is pushed.
  static final double MIN_BOOST = -0.7; // Will constantly boost if higher than 0
  static final double MAX_BOOST = 1.0;

  public DefaultDriveCommand(DriveSubsystem driveSubsystem, XboxController controller) {
    super(
        driveSubsystem.run(
            () ->
                driveSubsystem.driveWithExtras(
                    controller.getLeftY(),
                    -controller.getLeftX(), // ! THIS IS INVERTED BECAUSE +y IS LEFT of robot
                    -controller.getRightX(), // ! THIS IS INVERTED BECAUSE + IS COUNTERCLOCKWISE
                    getBoostByValue(controller))));
  }

  /**
   * Get the boost value for the drive command Boosts According to the shoulder analog inputs
   * doesn't go less than 0.2 for the convince of the driver
   *
   * @param controller
   * @return a double value between -0.8 and 1.0
   */
  private static double getBoostByValue(XboxController controller) {
    var boostBy = controller.getRightTriggerAxis();
    var reduceAmount = controller.getLeftTriggerAxis(); // between 0 and 1

    double totalBoost = boostBy - reduceAmount;

    totalBoost = MathUtil.clamp(totalBoost, MIN_BOOST, MAX_BOOST);
    return totalBoost;
  }
}
