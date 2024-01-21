package frc.robot.DataTypes;

public class PIDConstants {
  private double kP;
  private double kI;
  private double kD;
  private double kF;
  private double kIz;
  private double kMaxOutput;
  private double kMinOutput;

  PIDConstants(
      double kP,
      double kI,
      double kD,
      double kF,
      double kIz,
      double kMaxOutput,
      double kMinOutput) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.kIz = kIz;
    this.kMaxOutput = kMaxOutput;
    this.kMinOutput = kMinOutput;
  }

  public double getkP() {
    return kP;
  }

  public double getkI() {
    return kI;
  }

  public double getkD() {
    return kD;
  }

  public double getkF() {
    return kF;
  }

  public double getkIz() {
    return kIz;
  }

  public double getkMaxOutput() {
    return kMaxOutput;
  }

  public double getkMinOutput() {
    return kMinOutput;
  }
}
