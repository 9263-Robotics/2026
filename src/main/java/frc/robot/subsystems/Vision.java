package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class Vision extends SubsystemBase {
  PhotonCamera camera1 = new PhotonCamera("Camera-1");
  PhotonCamera camera2 = new PhotonCamera("Camera-2v2");


  public Field2d visionField = new Field2d();

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static PoseStrategy primaryStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  private static final double avgFactor = 16;
  
  // //TODO: add correct offsets to the estimators. need  cameras mounted tho.
  
  /** Creates a new ExampleSubsystem. */
  public Vision() { }

  public void updatePoseEstimation(SwerveDrive swerveDrive){
    for(Cameras curCamera: Cameras.values()) {
      curCamera.resultList = curCamera.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : curCamera.resultList) {
        double avg = distanceAvg(result) / avgFactor;
        if(result.getTargets().size() == 1) {
          curCamera.estimatedPose = curCamera.poseEstimator.estimateLowestAmbiguityPose(result);
          // swerveDrive.addVisionMeasurement(curCamera.estimatedPose.get().estimatedPose.toPose2d(), curCamera.estimatedPose.get().timestampSeconds);
          swerveDrive.addVisionMeasurement(curCamera.estimatedPose.get().estimatedPose.toPose2d(), curCamera.estimatedPose.get().timestampSeconds, VecBuilder.fill(0.5 * avg, 0.5 * avg, 1 * avg));
        } else if (result.getTargets().size() > 1) {
          curCamera.estimatedPose = curCamera.poseEstimator.estimateCoprocMultiTagPose(result);
          if (curCamera.estimatedPose.isPresent()){
            // swerveDrive.addVisionMeasurement(curCamera.estimatedPose.get().estimatedPose.toPose2d(), curCamera.estimatedPose.get().timestampSeconds);
            swerveDrive.addVisionMeasurement(curCamera.estimatedPose.get().estimatedPose.toPose2d(), curCamera.estimatedPose.get().timestampSeconds, VecBuilder.fill(0.5 * avg, 0.5 * avg, 1 * avg));
          }
        }
      }
    }
  }

  public double distanceAvg(PhotonPipelineResult result) {
    double value = 0.0;
    for(int i = 0; i < result.getTargets().size(); i++) 
      value += result.getTargets().get(i).getBestCameraToTarget().getTranslation().getNorm();
    return value / result.getTargets().size();
  }

  public enum Cameras {
    // BACK_RIGHT_CAM("Camera-1 (1)",
    //                 new Translation3d(-0.304, -0.304, 0.29), 
    //                 new Rotation3d(0,Math.toRadians(40),Math.toRadians(183))),
    //                 //40 vertical, 15 left (away from robot)
    // BACK_RIGHT_CAM("BackRightCam",
    //                 new Translation3d(-0.304, -0.304, 0.29), 
    //                 new Rotation3d(Math.toRadians(0),Math.toRadians(-40),Math.toRadians(15)).plus(new Rotation3d(0,0, Math.toRadians(180)))),
    //                 //40 vertical, 15 left (away from robot)
    
    // BACK_LEFT_CAM("BackLeftCam",
    //                 new Translation3d(-0.304, 0.304, 0.29), 
    //                 new Rotation3d(Math.toRadians(0),Math.toRadians(-40),Math.toRadians(-15)).plus(new Rotation3d(0,0,Math.toRadians(180)))),
    //                 //40 vertical, 15 left (away from robot)

    // LEFT_SIDE_CAM("FrontLeftCam",
    //                 new Translation3d(-0.0075, 0.3193, 0.41), 
    //                 new Rotation3d(0,0,Math.toRadians(70))),
    //                 //70 out
    
    // RIGHT_SIDE_CAM("FrontRightCam",
    //                 new Translation3d(-0.0075, -0.3193, 0.40), 
    //                 new Rotation3d(0,0,Math.toRadians(-70)));
    //                 //70 out

    LEFT_SIDE_CAM("FrontLeftCam",
                    new Translation3d(-0.045, 0.3145, 0.42), 
                    new Rotation3d(0,0,Math.toRadians(71.3)).plus(new Rotation3d(0,-0.8,0)));
                    //70 out
    
    // RIGHT_SIDE_CAM("FrontRightCam",
    //                 new Translation3d(-0.045, -0.3145, 0.40), 
    //                 new Rotation3d(0,0,Math.toRadians(-73.7)).plus(new Rotation3d(0,-1,0)));
    //                 //70 out

    // BACK_LEFT_CAM("Camera-1",
    //                 new Translation3d(-0, 0, 0), 
    //                 new Rotation3d(0,0,0));
                    
    
    
    
                 
    public PhotonCamera camera;
    public PhotonPoseEstimator poseEstimator;
    public Transform3d robotToCamTransform;
    public Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    public List<PhotonPipelineResult> resultList = new ArrayList<>();
                    
  
    Cameras(String name, Translation3d robotToCamTrans, Rotation3d robotToCamRot) {
      camera = new PhotonCamera(name);
      robotToCamTransform = new Transform3d(robotToCamTrans,robotToCamRot);
      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, Vision.primaryStrategy, robotToCamTransform);

    }
  }
}
