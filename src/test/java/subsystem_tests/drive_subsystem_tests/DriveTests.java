package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.drive_subsystem_tests.utils.DriveTestUtils;

class DriveTests extends DriveSubsystemTestBase {
  @BeforeEach
  public void setUp() {
    super.setUp();
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testRotation() {
    driveSubsystem.drive(0, 0, 0.5, false, false);

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0, 0, 0.5);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveChassisSpeed() {
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(2, 2, 0));

    var shouldBe = new ChassisSpeeds(2, 2, 0);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveForwardRobotRelative() {
    driveSubsystem.drive(0.5, 0, 0, false, false);

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0.5, 0, 0);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveSidewaysRobotRelative() {
    driveSubsystem.drive(0, 0.5, 0, false, false);

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0, 0.5, 0);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveForwardFieldRelative() {
    driveSubsystem.drive(0.5, 0, 0, true, false);

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(0), state.angle);
    }
  }

  @Test
  void testDriveSidewaysFieldRelative() {
    driveSubsystem.drive(0, 0.5, 0, true, false);

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(90), state.angle);
    }
  }

  @Test
  void testDriveDiagonalFieldRelative() {
    driveSubsystem.drive(1, 1, 0, true, false);

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(45), state.angle);
    }
  }

  @Test
  void testDriveDiagonalBackwardsFieldRelative() {
    driveSubsystem.drive(-1, -1, 0, true, false);

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(225), state.angle);
    }
  }
}
