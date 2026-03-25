package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {


    private final SwerveDrive swerveDrive;

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


    // command for zeroing the gyro, it needs disabling and re-enabling to start moving again after calling, might want to look into that
    public Command zeroGyro() {
        return run( () -> {
        swerveDrive.zeroGyro();
        });
    }
}