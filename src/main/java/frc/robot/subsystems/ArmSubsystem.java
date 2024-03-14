package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ArmPIDConstants;
import frc.utils.ExtraFunctions;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class ArmSubsystem extends SubsystemBase {
  // We only have RelativeEncoder for now. Its better than nothing.
  private final CANSparkMAXWrapped arm;
  private final CANSparkMAXWrapped armFollower;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final ArmFeedforward feedforward;

  public ArmSubsystem() {
    arm = new CANSparkMAXWrapped(IntakeConstants.kArmMotorCanID, MotorType.kBrushed);
    armFollower =
        new CANSparkMAXWrapped(IntakeConstants.kArmFollowerMotorCanID, MotorType.kBrushed);
    armFollower.follow(arm, true); // Inverted
    arm.restoreFactoryDefaults();
    encoder = arm.getEncoder(SparkRelativeEncoder.Type.kQuadrature, IntakeConstants.kArmEncoderCPR);
    pidController = arm.getPIDController();
    feedforward = new ArmFeedforward(ArmPIDConstants.kS, ArmPIDConstants.kG, ArmPIDConstants.kV);

    setupSparkMax();
  }

  /**
   * Sets up the Spark Max for the arm. This includes setting the PID constants, setting the
   * conversion factors for the encoder, setting the idle mode, and setting the smart current limit.
   * This method is here incase we want to switch to a different motor controller.
   */
  private void setupSparkMax() {
    arm.setInverted(false);
    encoder.setPositionConversionFactor(IntakeConstants.kArmEncoderPositionFactor);
    pidController.setFeedbackDevice(encoder);

    pidController.setP(IntakeConstants.ArmPIDConstants.kP);
    pidController.setI(IntakeConstants.ArmPIDConstants.kI);
    pidController.setD(IntakeConstants.ArmPIDConstants.kD);
    pidController.setFF(IntakeConstants.ArmPIDConstants.kFF);
    pidController.setOutputRange(-1, 1); // I don't think we need these as Constants in a file.

    arm.setIdleMode(IntakeConstants.kArmMotorIdleMode);
    arm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

    arm.burnFlash();
  }

  /**
   * Sets the arm to a position. This is a wrapper for the Spark Max's set method.
   *
   * @apiNote YOU SHOULD NOT USE THIS METHOD UNLESS NECESSARY! USE THE PID CONTROLLER INSTEAD.
   */
  public void setArmSpeed(double speed) {
    arm.set(speed);
  }

  /**
   * Sets the arm to a position. This is a wrapper for the Spark Max's set method.
   *
   * @apiNote YOU SHOULD NOT USE THIS METHOD UNLESS NECESSARY! USE THE PID CONTROLLER INSTEAD.
   */
  public void stopArm() {
    arm.set(0);
  }

  public boolean isArmAtPosition(double position) {
    return Math.abs(encoder.getPosition() - position) < ArmPIDConstants.kAllowedError;
  }

  /**
   * Sets the arm to a given position.
   *
   * @param position The rotation the arm should be at. (from 0 to 1)
   * @see #setArmToPosition(int)
   */
  public void setArmToPosition(double position) {
    pidController.setReference(
        position,
        ControlType.kPosition,
        0,
        feedforward.calculate(position, Math.abs(encoder.getPosition() - position)));
  }

  /**
   * Sets the arm to a given position.
   *
   * @param positionDegrees The rotation the arm should be at. (from 0 to 360)
   * @see #setArmToPosition(double)
   */
  public void setArmToPosition(int positionDegrees) {
    setArmToPosition(positionDegrees / 360.0);
  }

  public Command setArmToPositionCommand(double position) {
    return this.run(() -> this.setArmToPosition(position))
        .until(() -> this.isArmAtPosition(position));
  }

  /**
   * Sets the arm to a given position.
   *
   * @param positionDegrees The rotation the arm should be at. (from 0 to 360)
   * @return A command that will set the arm to the given position.
   */
  public Command setArmToPositionCommand(int positionDegrees) {
    return this.setArmToPositionCommand(positionDegrees / 360.0);
  }

  public void setArmToAprilTag() {
    var currentAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    boolean isBlueAlliance = currentAlliance == DriverStation.Alliance.Blue;
    int idToTrack = isBlueAlliance ? 7 : 4; // 7 is blue mid speaker, 4 is red mid speaker

    var target = PhotonCameraSystem.getAprilTagWithID(idToTrack);

    if (target.isEmpty()) {
      return;
    }

    double distance = target.get().getArea(); // will be from 0 to 100. (Hopefully?)

    double armAngle = ExtraFunctions.mapValue(distance, 0, 100, 0.2, 0.05); // between 10 and 20 deg

    setArmToPosition(armAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", encoder.getPosition());
  }
}
