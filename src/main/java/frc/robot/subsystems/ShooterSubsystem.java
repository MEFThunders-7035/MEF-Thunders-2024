package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  private final Spark shooterMotor;

  public ShooterSubsystem() {
    shooterMotor = new Spark(0);
  }

  @Override
  public void close() {
    shooterMotor.close();
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.setVoltage(speed * 12.0);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }
}
