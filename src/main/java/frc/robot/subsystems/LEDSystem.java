package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public final class LEDSystem {
  private static final LEDSubsystem ledSubsystemInstance = new LEDSubsystem();

  private LEDSystem() {}

  public static LEDSubsystem getInstance() {
    return ledSubsystemInstance;
  }

  public static Command run(Runnable toRun) {
    return ledSubsystemInstance.run(toRun);
  }

  public static Command getBlinkRedCommand() {
    return ledSubsystemInstance.getRedBlinkCommand();
  }
}
