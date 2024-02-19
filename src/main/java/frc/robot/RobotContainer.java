package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SmartShootCommand;
import frc.robot.commands.TestAutoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);

  private final SendableChooser<Command> autoChooser;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    PhotonCameraSystem.getAprilTagWithID(0); // Load the class before enable.
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(driveSubsystem.defaultDriveCommand(controller));
    intakeSubsystem.setDefaultCommand(
        new RunCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));

    shooterSubsystem.setDefaultCommand(
        new RunCommand(() -> shooterSubsystem.setShooterSpeed(0), shooterSubsystem));

    armSubsystem.setDefaultCommand(new RunCommand(armSubsystem::setArmToAprilTag, armSubsystem));
  }

  private void configureBindings() {
    new JoystickButton(controller, Button.kA.value) // Handbrake
        .whileTrue(new RunCommand(driveSubsystem::setX, driveSubsystem));

    new JoystickButton(controller, Button.kB.value) // Intake
        .whileTrue(
            new RunCommand(
                    () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed),
                    intakeSubsystem)
                .alongWith(intakeSubsystem.vibrateControllerOnNoteCommand(controller)));

    // TODO: DEPRECATED, REMOVE
    new JoystickButton(controller, Button.kX.value) // Shoot, basic (Run Shooter)
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.setShooterSpeed(ShooterConstants.kShooterSpeed),
                shooterSubsystem));

    new JoystickButton(controller, Button.kY.value) // Shoot, smart (Fully Shoot)
        .whileTrue(new SmartShootCommand(shooterSubsystem, intakeSubsystem));

    new JoystickButton(controller, Button.kStart.value) // Reset Heading
        .onTrue(
            driveSubsystem
                .runOnce(driveSubsystem::zeroHeading)
                .alongWith(new PrintCommand("Zeroing Heading")));

    // This command is here incase the intake gets stuck.
    new JoystickButton(controller, Button.kBack.value) // Force push note out of intake
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeSpeed(-IntakeConstants.kIntakeSpeed),
                intakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().andThen(() -> driveSubsystem.drive(0, 0, 0));
  }
}
