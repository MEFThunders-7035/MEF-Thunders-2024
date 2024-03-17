package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.SmartIntakeCommand;
import frc.robot.commands.SmartShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);

  private final SendableChooser<Command> autoChooser;
  
  private final GenericHID midiController = new GenericHID(1);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    loggingInit();
    configureJoystickBindings();
    setDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    PhotonCameraSystem.getAprilTagWithID(0); // Load the class before enable.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    if (RobotBase.isSimulation()) {
      simInit();
    }
  }

  private void loggingInit() {
    DataLogManager.start();
    URCL.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  private Thread simThread;

  public void simInit() {
    simThread =
        new Thread(
            () -> {
              System.out.println("Starting PhotonSim");
              while (true) {
                PhotonSim.update(new Pose2d());
                // I do not want to use a busy loop, so I added a delay.
                Timer.delay(0.2);
              }
            },
            "simThread");
    simThread.setDaemon(true);
    simThread.start();
  }

  public void simPeriodic() {
    if (!simThread.isAlive()) {
      simInit(); // If the thread dies, restart it.
      // This is here because sometimes the thread throws an exception and dies.
    }
    // add any simulation specific code here.
    // was made for photonSim, but it's not used.
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(driveSubsystem.defaultDriveCommand(controller));

    // move arm with midi's potentiometer
    armSubsystem.setDefaultCommand(
        new RunCommand(
            () -> armSubsystem.setArmToPosition(midiController.getRawAxis(0) / 2.0), armSubsystem));

    intakeSubsystem.setDefaultCommand(
        new RunCommand(
            () -> intakeSubsystem.setIntakeSpeed(midiController.getRawAxis(1)), intakeSubsystem));

    shooterSubsystem.setDefaultCommand(
        new RunCommand(
            () -> shooterSubsystem.setShooterSpeed(midiController.getRawAxis(2)),
            shooterSubsystem));
  }

  private void configureJoystickBindings() {
    new JoystickButton(controller, Button.kA.value) // Handbrake
        .whileTrue(new RunCommand(driveSubsystem::setX, driveSubsystem));

    new JoystickButton(controller, Button.kB.value) // Intake
        .whileTrue(new SmartIntakeCommand(intakeSubsystem, armSubsystem, controller));

    new JoystickButton(controller, Button.kY.value) // Shoot, smart (Fully Shoot)
        .whileTrue(new SmartShootCommand(shooterSubsystem, intakeSubsystem));

    new JoystickButton(controller, Button.kStart.value) // Reset Heading
        .onTrue(
            driveSubsystem
                .runOnce(driveSubsystem::zeroFieldOrientation)
                .alongWith(new PrintCommand("Zeroing Field Orientation"))
                .ignoringDisable(true));

    // This command is here incase the intake gets stuck.
    new JoystickButton(controller, Button.kBack.value) // Force push note out of intake
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeSpeed(-IntakeConstants.kIntakeSpeed),
                intakeSubsystem));

    new JoystickButton(controller, Button.kRightBumper.value) // Reverse Shooter to intake
        .whileTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(-1), shooterSubsystem));

    configureMidiBindings();
  }

  private void configureMidiBindings() {
    // This is incase the note detection system fails.
    new JoystickButton(midiController, 1) // Force Push intake on midi
        .whileTrue(intakeSubsystem.run(() -> intakeSubsystem.setIntakeSpeed(1, true)));

    new JoystickButton(midiController, 2)
        .onTrue(armSubsystem.runOnce(armSubsystem::resetEncoder).ignoringDisable(true));

    new JoystickButton(midiController, 16)
        .onTrue(
            driveSubsystem.runOnce(driveSubsystem::toggleForceRobotOriented).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().andThen(() -> driveSubsystem.drive(0, 0, 0));
  }
}
