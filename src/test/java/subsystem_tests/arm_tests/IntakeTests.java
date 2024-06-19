package subsystem_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTests {
  private IntakeSubsystem intakeSubsystem;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    intakeSubsystem = new IntakeSubsystem();
  }

  @AfterEach
  public void tearDown() {
    intakeSubsystem.close();
  }

  @Test
  void testIntakeSubsystem() {
    intakeSubsystem.setIntakeSpeed(0.5);
    assertEquals(0.5, intakeSubsystem.getArmIntakeSpeed(), 0.001);
  }

  @Test
  void testIntakeNoteDetected() {
    ColorSensorV3Wrapped.setNoteColor(true);
    assertEquals(true, intakeSubsystem.hasNote(), "Intake should detect a note");
    ColorSensorV3Wrapped.setNoteColor(false);
    assertEquals(false, intakeSubsystem.hasNote(), "Intake should not detect a note");
    // TODO: Add more tests for different RGBD values
  }

  @Test
  void testIntakeStopsOnNote() {
    ColorSensorV3Wrapped.setNoteColor(false);
    intakeSubsystem.setIntakeSpeed(0.5);
    assertEquals(
        0.5,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should be running when a note is not detected");
    ColorSensorV3Wrapped.setNoteColor(true);
    Timer.delay(0.1); // wait until note is detected
    assertEquals(
        0,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should stop when a note is detected");
  }

  @Test
  void testIntakeForcePushes() {
    ColorSensorV3Wrapped.setNoteColor(true);
    intakeSubsystem.setIntakeSpeed(0.5, 0, true); // force push note out
    assertEquals(
        0.5,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake Motor Should Run when force pushed");
    Timer.delay(0.15); // if broken, increase this
    assertEquals(
        0.5,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should continue to run when forced");
  }

  @Test
  void testIntakeSubsystemWithNote() {
    ColorSensorV3Wrapped.setNoteColor(true);
    intakeSubsystem.setIntakeSpeed(0.5);
    assertEquals(
        0,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should not be running when a note is detected");
  }
}
