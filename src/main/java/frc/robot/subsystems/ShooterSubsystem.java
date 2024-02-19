package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final Spark shooterMotor;

  public ShooterSubsystem() {
    shooterMotor = new Spark(0);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }
}
