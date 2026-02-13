package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class vision extends SubsystemBase {
  PhotonCamera camera1 = new PhotonCamera("Camera-1");
  PhotonCamera camera2 = new PhotonCamera("Camera-2v2");

  private Optional<EstimatedRobotPose> fieldToCamera1;
  private Optional<EstimatedRobotPose> fieldToCamera2;

  //TODO: add correct offsets to the estimators. need  cameras mounted tho.
  private PhotonPoseEstimator cam1Estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.195, -0.35, 0.25, new Rotation3d(0,0,0)));
  private PhotonPoseEstimator cam2Estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.2, 0.35, 0.25, new Rotation3d(0,0,0)));

  /** Creates a new ExampleSubsystem. */
  public vision() {
  }

  public void visionPoseUpdate(SwerveDrive m_SwerveDrive){
    // get pose from cam1-com2 & send it to the swerve module

    var cam1Pipeline = camera1.getAllUnreadResults();
    for (var result : cam1Pipeline){
      if (result.getTargets().size() == 1){
        fieldToCamera1 = cam1Estimator.estimateLowestAmbiguityPose(result);
        m_SwerveDrive.addVisionMeasurement(fieldToCamera2.get().estimatedPose.toPose2d(), fieldToCamera2.get().timestampSeconds);
        
      } else if (result.getTargets().size() > 1){
       fieldToCamera1 = cam1Estimator.estimateCoprocMultiTagPose(result);
       m_SwerveDrive.addVisionMeasurement(fieldToCamera2.get().estimatedPose.toPose2d(), fieldToCamera2.get().timestampSeconds);
      }
      if (result.getTargets().size() != 0){
      //  m_SwerveDrive.addVisionMeasurement(fieldToCamera1.get().estimatedPose.toPose2d(), fieldToCamera1.get().timestampSeconds, VecBuilder.fill(0.1,0.1,1));
      }
    }

    var cam2Pipeline = camera2.getAllUnreadResults();
    for (var result : cam2Pipeline){
      if (result.getTargets().size() == 1){
        fieldToCamera2 = cam2Estimator.estimateLowestAmbiguityPose(result);
        m_SwerveDrive.addVisionMeasurement(fieldToCamera2.get().estimatedPose.toPose2d(), fieldToCamera2.get().timestampSeconds);
      } else if (result.getTargets().size() > 1) {
       fieldToCamera2 = cam2Estimator.estimateCoprocMultiTagPose(result);
       m_SwerveDrive.addVisionMeasurement(fieldToCamera2.get().estimatedPose.toPose2d(), fieldToCamera2.get().timestampSeconds);
      }
      if (result.getTargets().size() != 0){
      //  m_SwerveDrive.addVisionMeasurement(fieldToCamera2.get().estimatedPose.toPose2d(), fieldToCamera2.get().timestampSeconds, VecBuilder.fill(0.1,0.1,1));
      }
    }
  }
}
