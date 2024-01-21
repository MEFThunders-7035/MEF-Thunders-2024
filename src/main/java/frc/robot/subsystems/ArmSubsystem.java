package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.utils.ExtraFunctions;

public class ArmSubsystem extends SubsystemBase {
  // We only have RelativeEncoder for now. Its better than nothing.
  private final CANSparkMax arm;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  public ArmSubsystem() {
    arm = new CANSparkMax(IntakeConstants.kArmMotorCanID, MotorType.kBrushed);
    encoder = arm.getEncoder(SparkRelativeEncoder.Type.kQuadrature, IntakeConstants.kArmEncoderCPR);
    pidController = arm.getPIDController();

    setupSparkMax();
  }

  /**
   * Sets up the Spark Max for the arm. This includes setting the PID constants, setting the
   * conversion factors for the encoder, setting the idle mode, and setting the smart current limit.
   * This method is here incase we want to switch to a different motor controller.
   */
  private void setupSparkMax() {

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
  public void setArmToZero() {
    arm.set(0);
  }

  /**
   * Sets the arm to a given position.
   *
   * @param position The rotation the arm should be at. (from 0 to 1)
   * @apiNote will be changed to an angle instead of a double. (360 degrees = 1 rotation)
   */
  // TODO: Change to an angle instead of a double. to: (360 degrees = 1 rotation)
  public void setArmToPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }

  public void setArmToAprilTag() {
    var currentAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    boolean isBlueAlliance = currentAlliance == DriverStation.Alliance.Blue;
    int idToTrack = isBlueAlliance ? 7 : 4; // 7 is blue mid speaker, 4 is red mid speaker

    var target = PhotonCameraSystem.getAprilTagWithID(idToTrack);

    if (target == null) {
      return;
    }

    double distance = target.getArea(); // will be from 0 to 100. (Hopefully?)

    double armAngle = ExtraFunctions.mapValue(distance, 0, 100, 0.1, 0.05); // between 10 and 20 deg

    pidController.setReference(armAngle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", encoder.getPosition());
  }
}
