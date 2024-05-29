package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.drive_subsystem_tests.utils.DriveTestUtils;

class DriveRateLimitTests extends DriveSubsystemTestBase {
  @BeforeEach
  public void setUp() {
    super.setUp();
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testForwardRateLimiter() {
    driveSubsystem.drive(0, 0, 0, false, false);

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0, 0, 0);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);

    driveSubsystem.drive(1, 0, 0, false, true);
    assertTrue(
        DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem).vxMetersPerSecond
            < DriveTestUtils.driveToChassisSpeeds(1, 0, 0).vxMetersPerSecond,
        "Rate Limit didn't get triggered!");
  }
}
