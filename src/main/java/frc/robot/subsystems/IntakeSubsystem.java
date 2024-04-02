package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ColorSensorConstants;
import frc.robot.commands.LoadToShooterCommand;
import frc.robot.commands.VibrateControllerCommand;
import frc.utils.sim_utils.CANSparkMAXWrapped;
import frc.utils.sim_utils.ColorSensorV3Wrapped;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped armIntake;
  private final CANSparkMAXWrapped groundIntake;
  private final ColorSensorV3Wrapped colorSensor =
      new ColorSensorV3Wrapped(ColorSensorConstants.kColorSensorPort);

  public IntakeSubsystem() {
    armIntake = new CANSparkMAXWrapped(IntakeConstants.kArmIntakeMotorCanID, MotorType.kBrushless);
    groundIntake =
        new CANSparkMAXWrapped(IntakeConstants.kGroundIntakeMotorCanID, MotorType.kBrushless);
    setupIntakeMotors();

    new Thread(
            () -> {
              while (true) {
                fastPeriodic();
              }
            },
            "Fast Color Check Loop")
        .start();
  }

  private void setupIntakeMotors() {
    armIntake.restoreFactoryDefaults();
    groundIntake.restoreFactoryDefaults();

    armIntake.setSmartCurrentLimit(20); // NEO 550 stall current is 20A
    groundIntake.setSmartCurrentLimit(20); // NEO 550 stall current is 20A
    armIntake.setInverted(true);
    groundIntake.setInverted(true);

    armIntake.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
    groundIntake.setIdleMode(IdleMode.kCoast);

    armIntake.burnFlash();
    groundIntake.burnFlash();
  }

  @Override
  public void close() {
    armIntake.close();
    groundIntake.close();
    colorSensor.close();
  }

  public boolean hasNote() {
    int red = colorSensor.getRed();
    int blue = colorSensor.getBlue();
    // If we are really close, we will decrease the threshold.
    // Proximity is inversely proportional to distance. and max of 2047 is 0 inches.
    if (colorSensor.getProximity() > 1600) {
      red = (int) (red * (1600.0 / colorSensor.getProximity()));
      blue = (int) (blue * (1600.0 / colorSensor.getProximity()));
    }

    return colorSensor.getProximity() > 300
        && red > 700
        && blue < 9000
        && red > colorSensor.getGreen();
  }

  public void setIntakeSpeed(double armSpeed, double groundSpeed, boolean force) {
    if (armSpeed > 0 && hasNote() && !force) { // If we are intaking, check if we have a note.
      armIntake.set(0);
    } else {
      armIntake.set(armSpeed);
    }

    groundIntake.set(groundSpeed);
  }

  public void setIntakeSpeed(double speed, boolean force) {
    if (speed > 0 && hasNote() && !force) { // If we are intaking, check if we have a note.
      armIntake.set(0);
      groundIntake.set(0);
      return;
    }

    armIntake.set(speed);
    groundIntake.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    setIntakeSpeed(speed, false);
  }

  public void checkIfHasNote() {
    if (hasNote() && armIntake.get() > 0) {
      setIntakeSpeed(0);
    }
  }

  public Command loadToShooterCommand() {
    return new LoadToShooterCommand(this);
  }

  public Command vibrateControllerOnNoteCommand(XboxController controller) {
    return new VibrateControllerCommand(controller, 2, 1, 0.2)
        .alongWith(new PrintCommand("Note!"))
        .beforeStarting(new WaitUntilCommand(this::hasNote));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ColorSensor - Distance", colorSensor.getProximity());
    SmartDashboard.putNumber("ColorSensor - Red", colorSensor.getRed());
    SmartDashboard.putNumber("ColorSensor - Green", colorSensor.getGreen());
    SmartDashboard.putNumber("ColorSensor - Blue", colorSensor.getBlue());
    SmartDashboard.putNumber("ColorSensor - IR", colorSensor.getIR());
    SmartDashboard.putBoolean("Note Detected", hasNote());
    checkIfHasNote();
  }

  private void fastPeriodic() {
    checkIfHasNote();
    Timer.delay(0.005);
  }
}
