package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.ExtraFunctions;

public class ShootToAmpCommand extends SequentialCommandGroup {
  public ShootToAmpCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      LEDSubsystem ledSubsystem,
      ArmSubsystem armSubsystem) {
    super(
        new ParallelRaceGroup(
            armSubsystem.moveToAmp(),
            new BasicRunShooterCommand(shooterSubsystem),
            ledSubsystem.blinkColor(ExtraFunctions.getAllianceColor())),
        Commands.race(intakeSubsystem.loadShooter(), new BasicRunShooterCommand(shooterSubsystem)),
        ledSubsystem.blinkColor(Color.kGreen));
  }
}
