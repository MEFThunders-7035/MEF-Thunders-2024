package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.DataTypes.CameraDetails;

public final class Constants {
  private Constants() {} // Prevents instantiation, as this is a utility class

  public static final class UtilConstants {
    public static int kRoborioDIOCount = 9;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = 0.71; // 71 cm between left and right wheels on robot
    public static final double kWheelBase = 0.79; // 79 cm between front and back wheels on robot

    public static final class MotorConstants {
      // Spark MAX CAN IDs
      public static final int kFrontLeftDrivingCanID = 1;
      public static final int kFrontLeftTurningCanID = 2;

      public static final int kFrontRightDrivingCanID = 3;
      public static final int kFrontRightTurningCanID = 4;

      public static final int kRearLeftDrivingCanID = 5;
      public static final int kRearLeftTurningCanID = 6;

      public static final int kRearRightDrivingCanID = 7;
      public static final int kRearRightTurningCanID = 8;
    }

    public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;
    }

    public static final class SwerveModuleConstants {
      // Angular offsets of the modules relative to the chassis in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;
      public static final SwerveDriveKinematics kDriveKinematics =
          new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in
    // a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps =
        DriveConstants.NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on
    // the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.02;
    public static final double kDriveSensitivity = 0.5; // The rest will be added by "boost"
    public static final double kDriveMaxOutput = 1.0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanID = 9;
    public static final int kArmMotorCanID = 10;
    public static final int kArmFollowerMotorCanID = 11;

    public static final IdleMode kArmMotorIdleMode = IdleMode.kCoast;

    public static final class ColorSensorConstants {
      public static final I2C.Port kColorSensorPort = I2C.Port.kMXP; // Connected to the NavX MXP
    }

    public static final int kArmEncoderCPR = 80; // The encoder isn't the best, but we will make do.
    public static final double kArmEncoderGearAmount = 1; // TODO: CHANGE THIS WHEN WE GET THE GEAR
    public static final double kArmEncoderPositionFactor = 1 / (kArmEncoderGearAmount);
    public static final int kSmartCurrentLimit = 40;

    public static final double kIntakeSpeed = 1;

    public static final class ArmPIDConstants {
      // TODO: Tune these values
      public static final double kP = 0.2;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kFF = 0.0;
    }
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPwmID = 0;

    public static final int kShooter1CanID = 11;
    public static final int kShooter2CanID = 12;

    public static final double kShooterSpeed = 0.6;
  }

  public static final class CameraConstants {
    public static final class PiCamera extends CameraDetails {
      public static final String cameraName = "piCamera";
      public static final double kCameraHeight = 0.0;
      public static final double kCameraDistanceMeters =
          0.45; // ~45 cm away from the center of the robot
      public static final double kCameraPitchRadians = Math.PI / 4; // 45 degree up
      public static final Transform3d robotToCam =
          new Transform3d(
              new Translation3d(kCameraDistanceMeters, kCameraHeight, 0),
              new Rotation3d(0, kCameraPitchRadians, 0));
    }
  }

  public static final class LEDConstants {
    public static final int kLedPin = 9;
    public static final int kLedCount = 50;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
