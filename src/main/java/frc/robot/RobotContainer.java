package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.simulationSystems.PhotonSim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.ExtraFunctions;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  private final GenericHID midiController = new GenericHID(1);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  public RobotContainer() {
    setupNamedCommands();
    loggingInit();
    configureJoystickBindings();
    setDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Shoot To Shooter", smartShootWithArm());
    PhotonCameraSystem.getAprilTagWithID(0); // Load the class before enable.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    if (RobotBase.isSimulation()) {
      simInit();
    }
    setupCamera();
  }

  private void setupCamera() {
    CameraServer.startAutomaticCapture();
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

  private Command intakeAndBlink() {
    return intakeSubsystem.intake().raceWith(ledSubsystem.blinkRed());
  }

  private Command shootToAmp() {
    return Commands.race(
            armSubsystem.moveToAmp(),
            shooterSubsystem.shoot(),
            ledSubsystem.blinkColor(ExtraFunctions.getAllianceColor()))
        .andThen(Commands.race(intakeSubsystem.loadShooter(), shooterSubsystem.shoot()))
        .andThen(ledSubsystem.blinkColor(Color.kGreen));
  }

  private Command smartShoot() {
    return Commands.sequence(
        Commands.deadline(ledSubsystem.loadingAnimation(2.5), shooterSubsystem.shoot())
            .andThen(shooterSubsystem.stop()),
        Commands.race(intakeSubsystem.loadShooter(), shooterSubsystem.shoot()));
  }

  private Command smartShootWithArm() {
    return Commands.race(armSubsystem.moveTo(driveSubsystem::getDistanceToShooter), smartShoot());
  }

  private void setupNamedCommands() {
    NamedCommands.registerCommand("Intake", intakeAndBlink());
    NamedCommands.registerCommand("Shoot To Speaker", smartShootWithArm());
    NamedCommands.registerCommand("Shoot To Amp", shootToAmp());
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(driveSubsystem.defaultDriveCommand(controller));

    // move arm with midi's potentiometer
    armSubsystem.setDefaultCommand((armSubsystem.moveTo(() -> midiController.getRawAxis(0))));

    intakeSubsystem.setDefaultCommand(
        new RunCommand(
            () -> intakeSubsystem.setIntakeSpeed(midiController.getRawAxis(1)), intakeSubsystem));

    shooterSubsystem.setDefaultCommand(shooterSubsystem.shoot(() -> midiController.getRawAxis(2)));

    ledSubsystem.setDefaultCommand(
        ledSubsystem.idle(intakeSubsystem::hasNote).ignoringDisable(true));
  }

  private void configureJoystickBindings() {
    controller.a().whileTrue(new RunCommand(driveSubsystem::setX, driveSubsystem));
    controller.b().whileTrue(intakeSubsystem.intakeThenVibrate(controller));
    controller
        .y()
        .whileTrue(
            Commands.parallel(
                driveSubsystem.driveFacingShooter(controller::getLeftY, controller::getLeftX),
                smartShootWithArm()));
    controller.x().whileTrue(shootToAmp());

    controller
        .start()
        .onTrue(
            driveSubsystem
                .runOnce(driveSubsystem::zeroFieldOrientation)
                .alongWith(new PrintCommand("Zeroing Field Orientation"))
                .ignoringDisable(true));
    controller.back().whileTrue(intakeSubsystem.eject());

    controller.leftBumper().whileTrue(smartShoot());
    controller.rightBumper().whileTrue(shooterSubsystem.intake());

    controller.pov(0).whileTrue(armSubsystem.moveToAmp());

    configureMidiBindings();
  }

  private void configureMidiBindings() {
    // This is incase load to shooter command fails.
    new JoystickButton(midiController, 1) // Force Push intake on midi
        .whileTrue(intakeSubsystem.run(() -> intakeSubsystem.setIntakeSpeed(1, true)));

    new JoystickButton(midiController, 2)
        .onTrue(armSubsystem.runOnce(armSubsystem::resetEncoder).ignoringDisable(true));

    new JoystickButton(midiController, 3).whileTrue(smartShoot());

    new JoystickButton(midiController, 4).whileTrue(smartShoot());

    new JoystickButton(midiController, 16)
        .onTrue(
            driveSubsystem.runOnce(driveSubsystem::toggleForceRobotOriented).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser
        .getSelected()
        .andThen(() -> driveSubsystem.drive(0, 0, 0))
        .beforeStarting(armSubsystem.runOnce(armSubsystem::resetEncoder));
  }
}
