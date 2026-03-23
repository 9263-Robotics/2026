package frc.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Vision;

public class StaticPoses {

    public static Optional<Pose2d> GetHubPoseOptional(){
        Optional<Pose2d> hubPose = Optional.empty();
        if (DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == Alliance.Red){
                // should be middle of red hub
                hubPose = Optional.of(Vision.fieldLayout.getTagPose(9).get().toPose2d().interpolate(Vision.fieldLayout.getTagPose(3).get().toPose2d(), 0.5));
            }
            else if (DriverStation.getAlliance().get() == Alliance.Blue){
                // should be middle of blue hub
                hubPose = Optional.of(Vision.fieldLayout.getTagPose(19).get().toPose2d().interpolate(Vision.fieldLayout.getTagPose(25).get().toPose2d(), 0.5));
            }
        }
        return hubPose;
    }

    public static Optional<Pose2d> GetScoringTableTrenchOptional(){
        Optional<Pose2d> trenchPose = Optional.empty();
        if (DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == Alliance.Blue){
                // should be blue trench on scoring table side
                trenchPose = Optional.of(Vision.fieldLayout.getTagPose(17).get().toPose2d());
            }
            else if (DriverStation.getAlliance().get() == Alliance.Red){
                // should be red trench on scoring table side
                trenchPose = Optional.of(Vision.fieldLayout.getTagPose(6).get().toPose2d());
            }
        }
        return trenchPose;
    }

    public static Optional<Pose2d> GetNonScoringTableTrenchOptional(){
        Optional<Pose2d> trenchPose = Optional.empty();
        if (DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == Alliance.Blue){
                // should be blue trench on non scoring table side
                trenchPose = Optional.of(Vision.fieldLayout.getTagPose(22).get().toPose2d());
            }
            else if (DriverStation.getAlliance().get() == Alliance.Red){
                // should be red trench on non scoring table side
                trenchPose = Optional.of(Vision.fieldLayout.getTagPose(1).get().toPose2d());
            }
        }
        return trenchPose;
    }
}
