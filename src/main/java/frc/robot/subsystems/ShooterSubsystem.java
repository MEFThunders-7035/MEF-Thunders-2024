package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  private final Spark shooterMotor;

  public ShooterSubsystem() {
    shooterMotor = new Spark(0);
  }

  @Override
  public void close() {
    shooterMotor.close();
  }

  public Command shoot(DoubleSupplier speed) {
    return run(() -> shooterMotor.set(ShooterConstants.kShooterSpeed));
  }

  public Command shoot() {
    return shoot(() -> ShooterConstants.kShooterSpeed);
  }

  public Command intake() {
    return shoot(() -> -1);
  }

  public Command stop() {
    return runOnce(shooterMotor::stopMotor);
  }
}
