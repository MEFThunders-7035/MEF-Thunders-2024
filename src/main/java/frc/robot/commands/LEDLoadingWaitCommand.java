package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;

public class LEDLoadingWaitCommand extends WaitCommand {
  private double seconds;
  private Color colorToFill;
  public static final Color DEFAULT_COLOR = new Color(0, 200, 255);
  private LEDSubsystem ledSubsystem;

  public LEDLoadingWaitCommand(LEDSubsystem ledSubsystem, double seconds, Color colorToFill) {
    super(seconds);
    this.seconds = seconds;
    this.colorToFill = colorToFill;
    this.ledSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
  }

  public LEDLoadingWaitCommand(LEDSubsystem ledSubsystem, double seconds) {
    this(ledSubsystem, seconds, DEFAULT_COLOR);
  }

  @Override
  public void execute() {
    // Wait command does not have an execute method so no need to call it.
    ledSubsystem.fillPercentageWithColor(m_timer.get() / seconds, colorToFill);
  }
}
