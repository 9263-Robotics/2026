package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.nio.channels.ShutdownChannelGroupException;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Turret extends SubsystemBase {

    private final SwerveSubsystem drivetrain;

    private final SparkFlex turretMotor = new SparkFlex(16, MotorType.kBrushless);
    private SparkFlexConfig turretMotorConfig = new SparkFlexConfig();

    private Pose2d turretPose = new Pose2d();
    private Rotation2d turretRotation = new Rotation2d();

    boolean badWait = false;

    // private final DigitalInput limitSwitch = new DigitalInput(0);
    // private boolean Zeroed;

    private DutyCycleEncoder absEncoder = new DutyCycleEncoder(9, 36,13.5);

    // private ProfiledPIDController turretPID = new ProfiledPIDController(0.25, 0.0, 0.0, new Constraints(800, 2500));
    private PIDController turretPID = new PIDController(0.25, 0.0, 0.0);

    private enum Targets {
        IDLE,
        HUB,
        PASSING
    }

    private Targets target = Targets.IDLE;

    private double desiredTurretAngle = 0;
    private double robotrotation = 0;

    private double calculatedRotation = 0;

    public Turret(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        turretMotorConfig.encoder.positionConversionFactor(360.0 / 50)
                                .velocityConversionFactor((360.0 / 50) / 60.0);

        turretMotorConfig.smartCurrentLimit(50);
        turretMotor.configure(turretMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        Shuffleboard.getTab(getName()).addDouble("Turret angle (robot relative)", this::getMotorEncoder);
        Shuffleboard.getTab(getName()).addDouble("Encoder value", absEncoder::get);
        Shuffleboard.getTab(getName()).addBoolean("Bad sleep", () -> badWait);
        // Shuffleboard.getTab(getName()).addDouble("Turret setpoint", () -> turretPID.getGoal().position);
        Shuffleboard.getTab(getName()).add("Turret PID", turretPID);
        Shuffleboard.getTab(getName()).addDouble("Turret Output PID", () -> -turretPID.calculate(turretMotor.getEncoder().getPosition()));

        Shuffleboard.getTab(getName()).addDouble("Turret Pose Rotation", () -> turretPose.getRotation().getDegrees());
        Shuffleboard.getTab(getName()).addDouble("Turret Global Rotation", () -> turretRotation.getDegrees());

        Shuffleboard.getTab(getName()).addDouble("robotrotation", () -> robotrotation);
        Shuffleboard.getTab(getName()).addDouble("calculatedRotation", () -> calculatedRotation);
        Shuffleboard.getTab(getName()).addDouble("Turret Desired Pos", () -> this.desiredTurretAngle);

        try{
            Thread.sleep(5000); //idk the encoder's not an early riser
        } catch (InterruptedException e){
            badWait = true;
        }
        double oldEncoder = absEncoder.get();
        turretMotor.getEncoder().setPosition(oldEncoder/*absEncoder.get()*/);

        turretPID.setTolerance(3);
        turretPID.setSetpoint(getMotorEncoder());

        Shuffleboard.getTab(getName()).addDouble("old encoder", () -> oldEncoder);

        // Rotation2d turretRotation = new Rotation2d(turretMotor.getEncoder().getPosition()).plus(drivetrain.getSwerveDrive().getPose().getRotation());
        
        turretPose = drivetrain.getSwerveDrive().getPose().plus(new Transform2d(-0.3,0.3, getTurretRotation()));

        desiredTurretAngle =  Rotation2d.fromDegrees(getMotorEncoder()).plus(drivetrain.getSwerveDrive().getPose().getRotation()).getDegrees();
    }

    @Override
    public void periodic(){
        if(DriverStation.isDisabled()){
            // turretPID.setGoal(getMotorEncoder());
            // desiredTurretAngle = turretRotation.getDegrees();
        }
        
        // if(DriverStation.getAlliance() )
        // robotrotation = ((drivetrain.getSwerveDrive().getPose().getRotation().getDegrees() < 0) ? (drivetrain.getSwerveDrive().getPose().getRotation().getDegrees() + 360) : (drivetrain.getSwerveDrive().getPose().getRotation().getDegrees()));
        // robotrotation = robotrotation % 180;

        robotrotation = drivetrain.getSwerveDrive().getPose().getRotation().getDegrees();
        calculatedRotation = desiredTurretAngle - robotrotation + (DegreesPerSecond.convertFrom(drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond, RadiansPerSecond) * 0.03);
        if (calculatedRotation < TurretConstants.minSoftStop) {
            calculatedRotation += 360;
        } else if (calculatedRotation > TurretConstants.maxSoftStop) {
            calculatedRotation -= 360;
        }

        if(calculatedRotation > TurretConstants.minSoftStop && calculatedRotation < TurretConstants.maxSoftStop){
            // turretPID.setSetpoint(MathUtil.clamp(calculatedRotation, -50, 50));
            turretPID.setSetpoint(calculatedRotation);
        }
        

        // turretPID.setSetpoint((desiredTurretAngle - drivetrain.getSwerveDrive().getGyro().getRotation3d().getAngle()) +  (drivetrain.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond) * 0.02)); // idk, getting the gyro dicrectly might fix the werid laggyness we were getting? and then accounting for the robot rotation could also make it track a bit better aswell (if we increase 0.02 it might track better in motion, but have a breif overshoot when we stop)

        // turretPID.setSetpoint(desiredTurretAngle - drivetrain.getSwerveDrive().getGyro().getRotation3d().getAngle());

        runPID();

        turretRotation = Rotation2d.fromDegrees(getMotorEncoder()).plus(drivetrain.getSwerveDrive().getPose().getRotation());

        turretPose = drivetrain.getSwerveDrive().getPose().plus(new Transform2d(-0.3,0.3, Rotation2d.fromDegrees(getMotorEncoder())));
    }

    public void setTurretAngle(Rotation2d rot) {
        if (rot.getDegrees() < -80 || rot.getDegrees() > 76)
            return;
        double delta1 = rot.getDegrees()/* - getTurretRotation().getDegrees()*/;
        delta1 = Math.IEEEremainder(delta1, 360);
        // turretPID.setGoal(delta1);
    }

    public void setTurretGlobalAngle(double desAngle) {
        desiredTurretAngle = desAngle;
    }

    public boolean isTurretAligned(){
        return turretPID.atSetpoint();
    }

    public Rotation2d getTurretRotation() {
        return turretRotation;
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    private double getMotorEncoder() {
        return -turretMotor.getEncoder().getPosition();
    }

    private void runPID(){
        // if(Zeroed){
        if (getMotorEncoder() < 90 && getMotorEncoder() > -90){
            // turretMotor.setVoltage(MathUtil.clamp(-turretPID.calculate(getMotorEncoder()), -5, 5));
            turretMotor.setVoltage(-turretPID.calculate(getMotorEncoder()));
        } else if (getMotorEncoder() > 90 && turretPID.calculate(getMotorEncoder())<0){
            // turretMotor.setVoltage(MathUtil.clamp(-turretPID.calculate(getMotorEncoder()), -5, 5));
            turretMotor.setVoltage(-turretPID.calculate(getMotorEncoder()));
        }else if (getMotorEncoder() < -90 && turretPID.calculate(getMotorEncoder())>0){
            // turretMotor.setVoltage(MathUtil.clamp(-turretPID.calculate(getMotorEncoder()), -5, 5));
            turretMotor.setVoltage(-turretPID.calculate(getMotorEncoder()));
        } else {
            turretMotor.setVoltage(0);
        }
    }

    // public Command zeroTurret() {
    //     return runEnd(() -> {
    //         turretMotor.setVoltage(0.1);
    //     }, () -> {
    //         turretMotor.set(0);
    //         turretMotor.getEncoder().setPosition(ScoringConstants.ZeroSwitchPos);
    //         turretPID.setGoal(turretMotor.getEncoder().getPosition());
    //         Zeroed = true;
    //     }).until(() -> limitSwitch.get());
    // }
}
