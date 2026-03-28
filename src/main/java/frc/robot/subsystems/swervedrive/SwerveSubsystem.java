package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
// import frc.robot.subsystems.Vision;

public class SwerveSubsystem extends SubsystemBase {


    private final SwerveDrive swerveDrive;
    // private final Vision m_vision = new Vision();

    public SwerveSubsystem(File directory){


        // Set Telemetry Verbosity (might want lower for comps as it can slow things down if it's too high, but for testing we don't care)
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try
        {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED, 
                                                                    new Pose2d(new Translation2d(Meter.of(1),
                                                                                Meter.of(4)),
                                                                                Rotation2d.fromDegrees(0)));
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        
        setUpPathplanner();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    // Methods for actually moving the motors, gets the velocity from the swerve input streams
    public void driveFieldOriented(ChassisSpeeds velcocity) {
        swerveDrive.driveFieldOriented(velcocity);
    }

    // A command that just runs the function above
    public Command driveFieldOriented(Supplier<ChassisSpeeds> Velocity){
        return run(()-> {
        swerveDrive.driveFieldOriented(Velocity.get());
        });
    }

    

    public void zeroGyroWithAlliance(){
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            swerveDrive.zeroGyro();

            swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }

        else {
            swerveDrive.zeroGyro();
        }
    }

    // command for zeroing the gyro, it needs disabling and re-enabling to start moving again after calling, might want to look into that
    public Command zeroGyro() {
        return run( () -> {
        swerveDrive.zeroGyro();
        });
    }
    
    @Override
    public void periodic() {
        // m_vision.updatePoseEstimation(swerveDrive);
    }

    public void setUpPathplanner() {
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;

            // Configure AutoBuilder last
            AutoBuilder.configure(
                swerveDrive::getPose, // Robot pose supplier

                swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)

                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                (speedsRobotRelative, moduleFeedForwards) -> { // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    if (enableFeedforward) {
                    swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                        );
                    } else {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                },
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
            );
            
        } catch (Exception e){
             e.printStackTrace();
        }

        
    }


     
}
