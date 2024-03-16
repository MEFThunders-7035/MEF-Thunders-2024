package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.MotorConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  private final AHRS navX = new AHRS();
  private Field2d field = new Field2d();

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
          Rotation2d.fromDegrees(navX.getAngle()),
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
    SmartDashboard.putNumber("Rotation", navX.getRotation2d().getDegrees());
    publisher.set(
        new SwerveModuleState[] {
          frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState(),
        });
  }

  private void updatePoseWithVision() {
    var poseOpt = PhotonCameraSystem.getEstimatedGlobalPose(field.getRobotPose());
    SmartDashboard.putBoolean("AprilTag Seen", poseOpt.isPresent());
    if (poseOpt.isPresent()) {
      swerveOdometry.addVisionMeasurement(
          poseOpt.get().estimatedPose.toPose2d(), poseOpt.get().timestampSeconds);
    }
    var shooterTarget = PhotonCameraSystem.getAprilTagWithID(7); // blue shooter
    SmartDashboard.putNumber(
        "Distance To Shooter",
        shooterTarget.isPresent()
            ? shooterTarget
                .get()
                .getBestCameraToTarget()
                .getTranslation()
                .toTranslation2d()
                .getDistance(new Translation2d())
            : 0);

    var tag = PhotonCameraSystem.getFieldLayout().getTagPose(7);
    if (tag.isEmpty()) return;
    SmartDashboard.putNumber(
        "Distance To Shooter 2",
        tag.get()
            .getTranslation()
            .toTranslation2d()
            .getDistance(swerveOdometry.getEstimatedPosition().getTranslation()));
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
        Rotation2d.fromDegrees(navX.getAngle()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
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
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, navX.getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    drive(xSpeed, ySpeed, rot, true, false);
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

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  public void zeroHeading() {
    navX.reset();
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

  /**
   * Drives the robot with the given Controller.
   *
   * <p>This is a convenience method for {@link #drive(double, double, double, double, double,
   * boolean, boolean)}. It's made to handle the deadband and boost.
   *
   * @param controller The controller to drive with.
   * @return The command to drive the robot.
   */
  public Command defaultDriveCommand(XboxController controller) {
    return run(
        () ->
            driveWithExtras(
                controller.getLeftY(),
                controller.getLeftX(),
                -controller.getRightX(),
                controller.getRightTriggerAxis()));
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
}
