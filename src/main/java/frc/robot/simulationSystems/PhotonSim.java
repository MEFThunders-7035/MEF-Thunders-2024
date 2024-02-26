package frc.robot.simulationSystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.PhotonCameraSystem;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonSim {
  // TODO: Implement PhotonSim
  private static PhotonCameraSim cameraSim;
  private static VisionSystemSim visionSim;

  public static void setupSim() {
    cameraSim = new PhotonCameraSim(PhotonCameraSystem.getCamera());
    visionSim = new VisionSystemSim(cameraSim.getCamera().getName());
    visionSim.addCamera(cameraSim, new Transform3d());
    visionSim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
  }

  public static void update(Pose2d robotPose) {
    visionSim.update(robotPose);
  }

  public static PhotonCameraSim getCameraSim() {
    return cameraSim;
  }

  public static void moveCameraSim(Pose2d pose) {
    visionSim.update(pose);
  }
}
