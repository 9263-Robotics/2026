// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import swervelib.SwerveDrive;

// public class Vision extends SubsystemBase {
//   PhotonCamera camera1 = new PhotonCamera("Camera-1");
//   PhotonCamera camera2 = new PhotonCamera("Camera-2v2");

//   public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

//   public static PoseStrategy primaryStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  
//   // //TODO: add correct offsets to the estimators. need  cameras mounted tho.
  
//   /** Creates a new ExampleSubsystem. */
//   public Vision() { }

//   public void updatePoseEstimation(SwerveDrive swerveDrive){
//     for(Cameras curCamera: Cameras.values()) {
//       curCamera.resultList = curCamera.camera.getAllUnreadResults();

//       for (PhotonPipelineResult result: curCamera.resultList) {
//         if(result.getTargets().size() == 1) {
//           curCamera.estimatedPose = curCamera.poseEstimator.estimateLowestAmbiguityPose(result);
//           swerveDrive.addVisionMeasurement(curCamera.estimatedPose.get().estimatedPose.toPose2d(), curCamera.estimatedPose.get().timestampSeconds);
//         } else if (result.getTargets().size() > 1) {
//           curCamera.estimatedPose = curCamera.poseEstimator.estimateCoprocMultiTagPose(result);
//           swerveDrive.addVisionMeasurement(curCamera.estimatedPose.get().estimatedPose.toPose2d(), curCamera.estimatedPose.get().timestampSeconds);
//         }
//       }
//     }
//   }

//   public enum Cameras {
//     FRONT_LEFT_CAM("Camera-1",
//                     new Translation3d(0.195, -0.35, 0.25), 
//                     new Rotation3d(0,0,0)),

//     FRONT_RIGHT_CAM("Camera-2v2",
//                     new Translation3d(0.2, 0.35, 0.25), 
//                     new Rotation3d(0,0,0));
    
    
    
                 
//     public PhotonCamera camera;
//     public PhotonPoseEstimator poseEstimator;
//     public Transform3d robotToCamTransform;
//     public Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
//     public List<PhotonPipelineResult> resultList = new ArrayList<>();
                    
  
//     Cameras(String name, Translation3d robotToCamTrans, Rotation3d robotToCamRot) {
//       camera = new PhotonCamera(name);
//       robotToCamTransform = new Transform3d(robotToCamTrans,robotToCamRot);
//       poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, Vision.primaryStrategy, robotToCamTransform);

//     }
//   }
// }
