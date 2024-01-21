package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// ! WARNING: This code will be changed to use 2 sparkMAXes with encoders and 1 being a follower.
public class ShooterSubsystem extends SubsystemBase {
  private final Spark shooterMotor;

  public ShooterSubsystem() {
    shooterMotor = new Spark(ShooterConstants.kShooterMotorPwmID);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }
}
