package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToAmp extends MoveArmToPositionCommand {
  public MoveArmToAmp(ArmSubsystem armSubsystem) {
    super(armSubsystem, ArmConstants.AMP_POSITION);
  }
}
