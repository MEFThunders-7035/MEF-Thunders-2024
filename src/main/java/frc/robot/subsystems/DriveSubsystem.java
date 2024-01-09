package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants.InterphaseConstants;


public class DriveSubsystem extends SubsystemBase{
    private AHRS navX;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private DifferentialDriveOdometry odometry;
    private Field2d field = new Field2d();
    
    
    public DriveSubsystem() {
        navX = new AHRS();
        leftEncoder = new Encoder(InterphaseConstants.kLeftEncoderPortA, InterphaseConstants.kLeftEncoderPortB);
        rightEncoder = new Encoder(InterphaseConstants.kRightEncoderPortA, InterphaseConstants.kRightEncoderPortB);
        odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
    }

    @Override
    public void periodic() {
        odometry.update(navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putNumber("Angle", navX.getAngle());
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
        SmartDashboard.putData(field);
    }
}
