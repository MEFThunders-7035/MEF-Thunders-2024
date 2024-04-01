package frc.robot.commands.led_commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSystem;

public class LEDLoadingWaitCommand extends WaitCommand {
  private double seconds;
  private Color colorToFill;

  public LEDLoadingWaitCommand(double seconds, Color colorToFill) {
    super(seconds);
    this.seconds = seconds;
    this.colorToFill = colorToFill;
    addRequirements(LEDSystem.getInstance());
  }

  public LEDLoadingWaitCommand(double seconds) {
    this(seconds, new Color(0, 200, 255));
  }

  @Override
  public void execute() {
    // Wait command does not have an execute method so no need to call it.
    LEDSystem.getInstance().fillPercentageWithColor(m_timer.get() / seconds, colorToFill);
  }
}
