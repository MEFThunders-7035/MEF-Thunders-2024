package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.RotationPIDController;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveFacingShooter extends SequentialCommandGroup {
  public DriveFacingShooter(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    super(getCommand(driveSubsystem, xSpeed, ySpeed));
  }

  private static PIDController getPIDController() {
    var controller =
        new PIDController(
            RotationPIDController.kP, RotationPIDController.kI, RotationPIDController.kD);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    return controller;
  }

  private static Command getCommand(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    PIDController rotController = getPIDController();
    rotController.setSetpoint(0);

    return driveSubsystem.run(
        () ->
            driveSubsystem.drive(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rotController.calculate(
                    driveSubsystem.getRotationDifferenceToShooter().getRadians())));
  }
}
