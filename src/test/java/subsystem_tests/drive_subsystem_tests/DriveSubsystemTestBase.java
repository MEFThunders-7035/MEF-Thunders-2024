package subsystem_tests.drive_subsystem_tests;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class DriveSubsystemTestBase {
  protected DriveSubsystem driveSubsystem;
  protected CommandScheduler commandScheduler;

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0);
    commandScheduler = CommandScheduler.getInstance();
    driveSubsystem = new DriveSubsystem();

    // Enable robot for commands to run
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
  }

  @AfterEach
  public void tearDown() {
    driveSubsystem.close();
    commandScheduler.cancelAll();
    commandScheduler.unregisterAllSubsystems(); // ! breaks all test tests if not done
    commandScheduler.close();
    HAL.shutdown();
  }
}
