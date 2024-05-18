package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToAmp extends MoveArmToPositionCommand {
  public MoveArmToAmp(ArmSubsystem armSubsystem) {
    super(armSubsystem, 0.5);
  }
}
