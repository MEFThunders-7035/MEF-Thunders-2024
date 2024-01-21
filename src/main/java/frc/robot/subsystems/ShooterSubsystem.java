package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// ! WARNING: This code will be changed to use 2 sparkMAXes with encoders and 1 being a follower.
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor1;
  private final CANSparkMax shooterMotor2;

  public ShooterSubsystem() {
    shooterMotor1 = new CANSparkMax(ShooterConstants.kShooter1CanID, MotorType.kBrushed);
    shooterMotor2 = new CANSparkMax(ShooterConstants.kShooter2CanID, MotorType.kBrushed);

    shooterMotor2.follow(shooterMotor1);
  }

  public void setShooterSpeed(double speed) {
    // USE PID CONTROLLER
  }
}
