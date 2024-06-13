package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PeriodicTests extends DriveSubsystemTestBase {
  @BeforeEach
  public void setUp() {
    super.setUp();
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testSimulationPeriodic() {
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(2, 0, 0));
    driveSubsystem.simulationPeriodic();

    assertEquals(2, SmartDashboard.getNumber("Front Left Swerve Speed", 0));
    assertEquals(2, SmartDashboard.getNumber("Front Right Swerve Speed", 0));
    assertEquals(2, SmartDashboard.getNumber("Rear Left Swerve Speed", 0));
    assertEquals(2, SmartDashboard.getNumber("Rear Right Swerve Speed", 0));

    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 2, 0));
    driveSubsystem.simulationPeriodic();

    assertEquals(90, SmartDashboard.getNumber("Front Left Swerve Rotation", 0));
    assertEquals(90, SmartDashboard.getNumber("Front Right Swerve Rotation", 0));
    assertEquals(90, SmartDashboard.getNumber("Rear Left Swerve Rotation", 0));
    assertEquals(90, SmartDashboard.getNumber("Rear Right Swerve Rotation", 0));
  }

  @Test
  void testPeriodic() {
    driveSubsystem.resetOdometry(new Pose2d(3, 2, Rotation2d.fromDegrees(30)));

    driveSubsystem.periodic();
    Field2d field = (Field2d) SmartDashboard.getData("Field");

    assertEquals(Rotation2d.fromDegrees(30), field.getRobotPose().getRotation());
    assertEquals(3, field.getRobotPose().getTranslation().getX());
    assertEquals(2, field.getRobotPose().getTranslation().getY());
  }
}
