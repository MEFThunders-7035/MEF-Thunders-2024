package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(driveSubsystem.defaultDriveCommand(controller));

    armSubsystem.setDefaultCommand(
        new RunCommand(() -> armSubsystem.setArmToAprilTag(), armSubsystem));
  }

  private void configureBindings() {
    new JoystickButton(controller, Button.kA.value)
        .whileTrue(new RunCommand(() -> driveSubsystem.setX(), driveSubsystem));

    new JoystickButton(controller, Button.kB.value)
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed),
                intakeSubsystem))
        .whileFalse(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));

    new JoystickButton(controller, Button.kX.value)
        .whileTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(0.6), shooterSubsystem))
        .whileFalse(new RunCommand(() -> shooterSubsystem.setShooterSpeed(0), shooterSubsystem));

    new JoystickButton(controller, Button.kStart.value)
        .onTrue(
            new RunCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem)
                .alongWith(new PrintCommand("Zeroing Heading")));
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveModuleConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            driveSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.SwerveModuleConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveSubsystem::setModuleStates,
            driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    // This is to ensure that odometry starts at the right place if we end up re-running this
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0));
  }
}
