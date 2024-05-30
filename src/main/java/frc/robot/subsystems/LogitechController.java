package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

public class LogitechController extends XboxController {
  public LogitechController(int port) {
    super(port);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @apiNote inverts axis value
   * @return The axis value.
   */
  @Override
  public double getLeftY() {
    return -super.getLeftY();
  }
}
