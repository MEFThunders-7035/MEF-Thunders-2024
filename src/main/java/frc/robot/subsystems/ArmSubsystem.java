package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ArmPIDConstants;
import frc.robot.commands.MoveArmToPositionCommand;
import frc.utils.ExtraFunctions;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  // We only have RelativeEncoder for now. Its better than nothing.
  private final CANSparkMAXWrapped arm;
  private final CANSparkMAXWrapped armFollower;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final ArmFeedforward feedforward;

  private static final double kArmParallelDifference = 0.00339;

  public ArmSubsystem() {
    arm = new CANSparkMAXWrapped(IntakeConstants.kArmMotorCanID, MotorType.kBrushed);
    armFollower =
        new CANSparkMAXWrapped(IntakeConstants.kArmFollowerMotorCanID, MotorType.kBrushed);
    armFollower.follow(arm, true); // Inverted
    arm.restoreFactoryDefaults();
    encoder = arm.getEncoder(SparkRelativeEncoder.Type.kQuadrature, IntakeConstants.kArmEncoderCPR);
    pidController = arm.getPIDController();
    feedforward = new ArmFeedforward(ArmPIDConstants.kS, ArmPIDConstants.kG, ArmPIDConstants.kV);

    SmartDashboard.putNumber("Arm Pos", 0);
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
    pidController.setIMaxAccum(0.1, 0);
    pidController.setOutputRange(-0.2, 0.7); // I don't think we need these as Constants in a file.

    arm.setIdleMode(IntakeConstants.kArmMotorIdleMode);
    arm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

    arm.burnFlash();
  }

  @Override
  public void close() {
    arm.close();
    armFollower.close();
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
    position = MathUtil.clamp(position, 0, 0.5);
    return Math.abs(encoder.getPosition() - position) < ArmPIDConstants.kAllowedError;
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public void resetEncoder(double position) {
    encoder.setPosition(position);
  }

  /**
   * Sets the arm to a given position.
   *
   * @param position The rotation the arm should be at. (from 0 to 1)
   * @see #setArmToPosition(int)
   */
  public void setArmToPosition(double position) {
    // the hand is slightly down, which means the 0 value is
    // actually "kArmParallelDifference" down so we compensate for that in the calculation
    position = MathUtil.clamp(position, 0, 0.5);
    var calculation =
        feedforward.calculate(
            (position - kArmParallelDifference) * Math.PI,
            Math.abs(encoder.getPosition() - position));
    SmartDashboard.putNumber("FeedForward Calculation", calculation);
    SmartDashboard.putNumber("Setpoint", position);
    pidController.setReference(
        position, ControlType.kPosition, 0, calculation, ArbFFUnits.kVoltage);
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
    return new MoveArmToPositionCommand(this, position);
  }

  /**
   * Sets the arm to a given position.
   *
   * @param positionDegrees The rotation the arm should be at. (from 0 to 360)
   * @return A command that will set the arm to the given position.
   */
  public Command setArmToPositionCommand(int positionDegrees) {
    return this.setArmToPositionCommand(positionDegrees / 180.0);
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
