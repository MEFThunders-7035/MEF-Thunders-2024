package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants.DrivePIDController;
import frc.robot.Constants.AutoConstants.RotationPIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.MotorConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.ExtraFunctions;
import frc.utils.SwerveUtils;
import java.util.Optional;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  private final AHRS navX = new AHRS();
  private Field2d field = new Field2d();

  private Rotation2d fieldOrientationRotateBy = new Rotation2d();
  private boolean forceRobotOriented = false;

  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          MotorConstants.kFrontLeftDrivingCanID,
          MotorConstants.kFrontLeftTurningCanID,
          SwerveModuleConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          MotorConstants.kFrontRightDrivingCanID,
          MotorConstants.kFrontRightTurningCanID,
          SwerveModuleConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          MotorConstants.kRearLeftDrivingCanID,
          MotorConstants.kRearLeftTurningCanID,
          SwerveModuleConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          MotorConstants.kRearRightDrivingCanID,
          MotorConstants.kRearRightTurningCanID,
          SwerveModuleConstants.kBackRightChassisAngularOffset);

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  SwerveDrivePoseEstimator swerveOdometry =
      new SwerveDrivePoseEstimator(
          SwerveModuleConstants.kDriveKinematics,
          getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          },
          new Pose2d());

  StructArrayPublisher<SwerveModuleState> publisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Swerve", SwerveModuleState.struct)
          .publish();

  public DriveSubsystem() {
    // Do nothing
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(
                DrivePIDController.kP, DrivePIDController.kD), // Translation PID constants
            new PIDConstants(
                RotationPIDController.kP, RotationPIDController.kD), // Rotation PID constants
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            0.55, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(
                true, true) // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
    SmartDashboard.putNumber("Move By", 0);
    if (RobotBase.isSimulation()) {
      SmartDashboard.putNumber("X position", 0);
      SmartDashboard.putNumber("Y position", 0);
    }
  }

  @Override
  public void periodic() {
    var pose =
        swerveOdometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              rearLeft.getPosition(),
              rearRight.getPosition()
            });
    field.setRobotPose(pose);
    updatePoseWithVision();
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Rotation", getHeading());
    SmartDashboard.putBoolean("Force Robot Oriented", forceRobotOriented);
    if (RobotBase.isReal()) {
      publisher.set(
          new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState(),
          });
    }
  }

  @Override
  public void simulationPeriodic() {
    var states = getModuleDesiredStates();
    publisher.set(states);
    SmartDashboard.putNumber("Front Left Swerve Speed", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left Swerve Rotation", states[0].angle.getDegrees());
    SmartDashboard.putNumber("Front Right Swerve Speed", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Swerve Rotation", states[1].angle.getDegrees());
    SmartDashboard.putNumber("Rear Left Swerve Speed", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Rear Left Swerve Rotation", states[2].angle.getDegrees());
    SmartDashboard.putNumber("Rear Right Swerve Speed", states[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Rear Right Swerve Rotation", states[3].angle.getDegrees());

    var xPos = SmartDashboard.getNumber("X position", 0);
    var yPos = SmartDashboard.getNumber("Y position", 0);

    if (xPos != 0 && yPos != 0) {
      resetOdometry(new Pose2d(xPos, yPos, getRotation2d()));
    }
  }

  @Override
  public void close() {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();

    field.close();
    publisher.close();
  }

  private void updatePoseWithVision() {
    var poseOpt = PhotonCameraSystem.getEstimatedGlobalPose(field.getRobotPose());
    SmartDashboard.putBoolean("AprilTag Seen", poseOpt.isPresent());
    if (poseOpt.isPresent()) {
      swerveOdometry.addVisionMeasurement(
          poseOpt.get().estimatedPose.toPose2d(), poseOpt.get().timestampSeconds);
    }

    SmartDashboard.putNumber("Distance To Shooter", getDistanceToShooter());

    SmartDashboard.putNumber(
        "Rotation Difference to Shooter", getRotationDifferenceToShooter().getDegrees());

    SmartDashboard.putNumber(
        "Arm Angle Required", ExtraFunctions.getAngleFromDistance(getDistanceToShooter()));
  }

  private Optional<Pose3d> getTagPose(int id) {
    var tag = PhotonCameraSystem.getFieldLayout().getTagPose(id);

    if (tag.isEmpty()) {
      DriverStation.reportError("Field Layout Couldn't be loaded", false);
      return Optional.empty();
    }

    return tag;
  }

  /**
   * Gets the distance to the shooter.
   *
   * @exception DriverStation.reportError if the field layout couldn't be loaded. and returns 0.
   * @return returns the distance to the shooter in meters. will return 0 if the field layout
   *     couldn't be loaded.
   */
  public double getDistanceToShooter() {
    var tag = getTagPose(ExtraFunctions.getShooterAprilTagID());
    if (tag.isEmpty()) return 0;

    return tag.get().getTranslation().toTranslation2d().getDistance(getPose().getTranslation())
        + SmartDashboard.getNumber("Move By", 0);
  }

  public Rotation2d getRotationDifferenceToShooter() {
    var tag = getTagPose(ExtraFunctions.getShooterAprilTagID());

    if (tag.isEmpty()) return new Rotation2d();

    return tag.get()
        .getTranslation()
        .toTranslation2d()
        .minus(getPose().getTranslation())
        .getAngle()
        .minus(getRotation2d())
        .rotateBy(Rotation2d.fromDegrees(180))
        .times(-1);
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void setForceRobotOriented(boolean forceRobotOriented) {
    this.forceRobotOriented = forceRobotOriented;
  }

  public void toggleForceRobotOriented() {
    forceRobotOriented = !forceRobotOriented;
  }

  public boolean isForceRobotOriented() {
    return forceRobotOriented;
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveModuleConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
  }

  /**
   * Method to drive the robot using joystick info. Taken straight from REV MAXSwerve Template.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (forceRobotOriented) {
      fieldRelative = false;
    }

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively
        // instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality
          // checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        SwerveModuleConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getFieldOrientedRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = SwerveModuleConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    drive(xSpeed, ySpeed, rot, true, true);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleDesiredStates() {
    return new SwerveModuleState[] {
      frontLeft.getDesiredState(),
      frontRight.getDesiredState(),
      rearLeft.getDesiredState(),
      rearRight.getDesiredState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   *
   * @apiNote THIS SHOULD NOT BE USED IN A MATCH, USE ZERO FIELD ORIENTATION INSTEAD ONLY USE THIS
   *     IN AN EMERGENCY AS IT WILL FUCK UP THE VISION AND OTHER ODOMETRY SYSTEMS
   */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * Zeros the heading for the field orientation system, so that it is easier to use looking trough
   * a different orientation.
   *
   * @see {@link zeroHeading} for actually resetting the heading on the hardware level
   */
  public void zeroFieldOrientation() {
    fieldOrientationRotateBy = getRotation2d().unaryMinus().rotateBy(Rotation2d.fromDegrees(0));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navX.getAngle();
  }

  public Rotation2d getRotation2d() {
    return navX.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate();
  }

  /** This will handle adding Deadband, and adding boost to the drive. */
  public void driveWithExtras(
      double xSpeed,
      double ySpeed,
      double rot,
      double boost,
      double deadband,
      boolean fieldRelative,
      boolean rateLimit) {
    final double sens = OIConstants.kDriveSensitivity; // The rest will be added by "boost"
    drive(
        MathUtil.applyDeadband(xSpeed * (sens + (boost * (1 - sens))), deadband),
        MathUtil.applyDeadband(ySpeed * (sens + (boost * (1 - sens))), deadband),
        MathUtil.applyDeadband(rot * sens, deadband),
        fieldRelative,
        rateLimit);
    SmartDashboard.putNumber("xSpeed: ", xSpeed);
    SmartDashboard.putNumber("ySpeed: ", ySpeed);
  }

  public void driveWithExtras(
      double xSpeed, double ySpeed, double rot, double boost, double deadband) {
    driveWithExtras(xSpeed, ySpeed, rot, boost, deadband, true, true);
  }

  public void driveWithExtras(double xSpeed, double ySpeed, double rot, double boost) {
    driveWithExtras(xSpeed, ySpeed, rot, boost, OIConstants.kDriveDeadband);
  }

  private Rotation2d getFieldOrientedRotation2d() {
    return getRotation2d().rotateBy(fieldOrientationRotateBy);
  }
}
