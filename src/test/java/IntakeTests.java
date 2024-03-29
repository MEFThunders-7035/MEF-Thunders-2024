import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import frc.utils.sim_utils.SparkMAXSimAddon;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTests {
  private IntakeSubsystem intakeSubsystem;
  private CANSparkMax intakeMotor;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    intakeSubsystem = new IntakeSubsystem();
    intakeMotor = SparkMAXSimAddon.getSparkMAX(IntakeConstants.kArmIntakeMotorCanID);
  }

  @AfterEach
  public void tearDown() {
    intakeSubsystem.close();
  }

  @Test
  void testIntakeSubsystem() {
    intakeSubsystem.setIntakeSpeed(0.5);
    assertEquals(0.5, intakeMotor.get(), 0.001);
  }

  @Test
  void testIntakeNoteDetected() {
    ColorSensorV3Wrapped.setRGBD(2500, 0, 0, 900);
    assertEquals(true, intakeSubsystem.hasNote(), "Intake should detect a note");
    ColorSensorV3Wrapped.setRGBD(0, 0, 0, 0);
    assertEquals(false, intakeSubsystem.hasNote(), "Intake should not detect a note");
    // TODO: Add more tests for different RGBD values
  }

  @Test
  void testIntakeStopsOnNote() {
    ColorSensorV3Wrapped.setRGBD(0, 0, 0, 0);
    intakeSubsystem.setIntakeSpeed(0.5);
    assertEquals(
        0.5,
        intakeMotor.get(),
        0.001,
        "Intake motor should be running when a note is not detected");
    ColorSensorV3Wrapped.setRGBD(2500, 0, 0, 900);
    Timer.delay(0.1);
    assertEquals(0, intakeMotor.get(), 0.001, "Intake motor should stop when a note is detected");
  }

  @Test
  void testIntakeSubsystemWithNote() {
    ColorSensorV3Wrapped.setRGBD(2500, 0, 0, 900);
    intakeSubsystem.setIntakeSpeed(0.5);
    assertEquals(
        0, intakeMotor.get(), 0.001, "Intake motor should not be running when a note is detected");
  }
}
