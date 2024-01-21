package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ColorSensorConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake;
  private final ColorSensorV3 colorSensor =
      new ColorSensorV3(ColorSensorConstants.kColorSensorPort);

  public IntakeSubsystem() {
    intake = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
  }

  public boolean hasNote() {
    return colorSensor.getRed() > 150 && colorSensor.getBlue() < 150;
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

  public Command loadToShooter() {
    class loadShooterCommand extends Command {
      loadShooterCommand() {
        addRequirements(IntakeSubsystem.this);
      }

      @Override
      public void initialize() {
        setIntakeSpeed(IntakeConstants.kIntakeSpeed);
      }

      @Override
      public boolean isFinished() {
        return !hasNote();
      }

      @Override
      public void end(boolean interrupted) {
        setIntakeSpeed(0);
      }
    }

    return new loadShooterCommand();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ColorSensor - Distance", colorSensor.getProximity());
    SmartDashboard.putNumber("ColorSensor - Red", colorSensor.getRed());
    SmartDashboard.putNumber("ColorSensor - Green", colorSensor.getGreen());
    SmartDashboard.putNumber("ColorSensor - Blue", colorSensor.getBlue());
    SmartDashboard.putNumber("ColorSensor - IR", colorSensor.getIR());

    checkIfHasNote();
  }
}
