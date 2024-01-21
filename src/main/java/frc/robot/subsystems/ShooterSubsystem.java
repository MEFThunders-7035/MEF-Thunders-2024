package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// ! WARNING: This code will be changed to use 2 sparkMAXes with encoders and 1 being a follower.
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor1;
  private final CANSparkMax shooterMotor2;

  private final RelativeEncoder m_shooterMotor1Encoder;
  private final RelativeEncoder m_shooterMotor2Encoder;

  private final SparkPIDController m_shooterMotor1PIDController;
  private final SparkPIDController m_shooterMotor2PIDController;

  public ShooterSubsystem() {
    shooterMotor1 = new CANSparkMax(ShooterConstants.kShooter1CanID, MotorType.kBrushed);
    m_shooterMotor1Encoder =
        shooterMotor1.getEncoder(
            SparkRelativeEncoder.Type.kQuadrature, ShooterConstants.kShooterEncoderCPR);
    m_shooterMotor1PIDController = shooterMotor1.getPIDController();

    shooterMotor2 = new CANSparkMax(ShooterConstants.kShooter2CanID, MotorType.kBrushed);
    m_shooterMotor2Encoder =
        shooterMotor2.getEncoder(
            SparkRelativeEncoder.Type.kQuadrature, ShooterConstants.kShooterEncoderCPR);
    m_shooterMotor2PIDController = shooterMotor2.getPIDController();

    shooterMotor2.follow(shooterMotor1);

    setupMotor1SparkMax();
    setupMotor2SparkMax();
  }

  private void setupMotor1SparkMax() {
    m_shooterMotor1Encoder.setPositionConversionFactor(
        ShooterConstants.kShooterEncoderPositionFactor);
    m_shooterMotor1PIDController.setFeedbackDevice(m_shooterMotor1Encoder);

    m_shooterMotor1PIDController.setP(ShooterConstants.ShooterPIDConstants.kP);
    m_shooterMotor1PIDController.setI(ShooterConstants.ShooterPIDConstants.kI);
    m_shooterMotor1PIDController.setD(ShooterConstants.ShooterPIDConstants.kD);
    m_shooterMotor1PIDController.setFF(ShooterConstants.ShooterPIDConstants.kFF);
    m_shooterMotor1PIDController.setOutputRange(-1, 1);

    shooterMotor1.burnFlash();
  }

  private void setupMotor2SparkMax() {
    m_shooterMotor2Encoder.setPositionConversionFactor(
        ShooterConstants.kShooterEncoderPositionFactor);
    m_shooterMotor2PIDController.setFeedbackDevice(m_shooterMotor2Encoder);

    m_shooterMotor2PIDController.setP(ShooterConstants.ShooterPIDConstants.kP);
    m_shooterMotor2PIDController.setI(ShooterConstants.ShooterPIDConstants.kI);
    m_shooterMotor2PIDController.setD(ShooterConstants.ShooterPIDConstants.kD);
    m_shooterMotor2PIDController.setFF(ShooterConstants.ShooterPIDConstants.kFF);
    m_shooterMotor2PIDController.setOutputRange(-1, 1);

    shooterMotor2.burnFlash();
  }

  public void setShooterSpeed(double speed) {
    m_shooterMotor1PIDController.setReference(speed, ControlType.kVelocity);
  }
}
