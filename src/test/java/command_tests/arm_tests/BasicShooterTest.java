package command_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicRunShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BasicShooterTest extends CommandTestBase {
  private ShooterSubsystem shooterSubsystem;
  private PWMSim shooterMotor;
  private BasicRunShooterCommand basicShooterCommand;
  private static final double waitTime = 2;

  @BeforeEach
  public void setUp() {
    super.setUp();
    shooterSubsystem = new ShooterSubsystem();
    shooterMotor = new PWMSim(ShooterConstants.kShooterMotorPwmID);
    basicShooterCommand = new BasicRunShooterCommand(shooterSubsystem, waitTime);

    commandScheduler.schedule(basicShooterCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
    shooterSubsystem.close();
  }

  @Test
  void testSetSpeed() {
    commandScheduler.run();
    // Motor should be at the set speed
    assertEquals(
        ShooterConstants.kShooterSpeed,
        shooterMotor.getSpeed(),
        "Shooter motor should be at shooter speed");
    Timer.delay(waitTime);
    commandScheduler.run();
    // Motor should be at 0 after waitTime
    assertEquals(
        0,
        shooterMotor.getSpeed(),
        "Shooter motor should be at 0 after %s seconds".formatted(waitTime));
  }

  // @Test
  // void testLEDLoading() {
  //   commandScheduler.run();
  //   Timer.delay(waitTime - 0.2); // let led loop do its thing
  //   commandScheduler.run();
  //   // LED should be green after waitTime
  //   checkForColorInAll(
  //       ledSubsystem, LEDLoadingWaitCommand.DEFAULT_COLOR, "Color should be green after
  // waitTime");
  // }
}
