package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class vision extends SubsystemBase {
  PhotonCamera camera1 = new PhotonCamera("Arducam_OV9281_USB_Camera");
  PhotonCamera camera2 = new PhotonCamera("Camera2v2");

  private Optional<EstimatedRobotPose> fieldToCamera1;
  private Optional<EstimatedRobotPose> fieldToCamera2;

  //TODO: add correct offsets to the estimators. need  cameras mounted tho.
  private PhotonPoseEstimator cam1Estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0, 0, 0, new Rotation3d(0,0,0)));
  private PhotonPoseEstimator cam2Estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0, 0, 0, new Rotation3d(0,0,0)));

  /** Creates a new ExampleSubsystem. */
  public vision() {
    cam1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
  }

  public void visionPoseUpdate(SwerveDrive m_SwerveDrive){
    // get pose from cam1-com2 & send it to the swerve module

    var cam1Pipeline = camera1.getAllUnreadResults();
    for (var result : cam1Pipeline){
      fieldToCamera1 = cam1Estimator.estimateCoprocMultiTagPose(result);
      m_SwerveDrive.addVisionMeasurement(fieldToCamera1.get().estimatedPose.toPose2d(), fieldToCamera1.get().timestampSeconds);
    }

    var cam2Pipeline = camera2.getAllUnreadResults();
    for (var result : cam2Pipeline){
       fieldToCamera2 = cam2Estimator.estimateCoprocMultiTagPose(result);

       m_SwerveDrive.addVisionMeasurement(fieldToCamera2.get().estimatedPose.toPose2d(), fieldToCamera2.get().timestampSeconds);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
