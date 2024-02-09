package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ColorSensorConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake;
  private final ColorSensorV3 colorSensor =
      new ColorSensorV3(ColorSensorConstants.kColorSensorPort);

  public IntakeSubsystem() {
    intake = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushed);
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

    return colorSensor.getProximity() > 700
        && red > 2000
        && blue < 2500
        && red > colorSensor.getGreen();
  }

  public void setIntakeSpeed(double speed) {
    if (speed > 0 && hasNote()) { // If we are intaking, check if we have a note.
      intake.set(0);
      return;
    }

    intake.set(speed);
  }

  public void checkIfHasNote() {
    if (hasNote() && intake.get() > 0) {
      intake.set(0);
    }
  }

  public Command loadToShooterCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> intake.set(IntakeConstants.kIntakeSpeed),
        interrupted -> setIntakeSpeed(0),
        () -> !hasNote(),
        this);
  }

  public Command vibrateControllerOnNoteCommand(XboxController controller) {
    return new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0.5))
        .andThen(new WaitCommand(0.5))
        .andThen(() -> controller.setRumble(RumbleType.kBothRumble, 0))
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
}
