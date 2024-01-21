package frc.robot.DataTypes;

import edu.wpi.first.math.geometry.Transform3d;

public abstract class CameraDetails {
  protected String cameraName = "TestCamera";
  protected double kCameraHeight = 0.0;
  protected double kCameraPitchRadians = Math.PI / 4;
  protected Transform3d robotToCam = new Transform3d();

  public String getCameraName() {
    return cameraName;
  }

  public double getCameraHeight() {
    return kCameraHeight;
  }

  public double getCameraPitchRadians() {
    return kCameraPitchRadians;
  }

  public Transform3d getRobotToCam() {
    return robotToCam;
  }
}
