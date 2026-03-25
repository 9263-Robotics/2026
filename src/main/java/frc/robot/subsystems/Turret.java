package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.CANIDs.TurretPID;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Turret extends SubsystemBase {

    private final SwerveSubsystem drivetrain;

    private final SparkFlex turretMotor = new SparkFlex(CANIDs.TurretMotor, MotorType.kBrushless);
    private SparkFlexConfig turretMotorConfig = new SparkFlexConfig();

    private Pose2d turretPose = new Pose2d();
    private Rotation2d turretRotation = new Rotation2d();

    // private final DigitalInput limitSwitch = new DigitalInput(0);
    // private boolean Zeroed;

    private DutyCycleEncoder absEncoder = new DutyCycleEncoder(0, 36,0);

    private ProfiledPIDController turretPID = new ProfiledPIDController(TurretPID.k, TurretPID.i, TurretPID.d, new Constraints(TurretPID.maxVel, TurretPID.maxAccel));

    private enum Targets {
        IDLE,
        HUB,
        PASSING
    }

    private Targets target = Targets.IDLE;

    public Turret(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        turretMotorConfig.encoder.positionConversionFactor(360.0 / 50)
                                .velocityConversionFactor((360.0 / 50) / 60.0);

        turretMotorConfig.smartCurrentLimit(50);
        turretMotor.configure(turretMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        turretMotor.getEncoder().setPosition(absEncoder.get());

        turretPID.setTolerance(3);
        turretPID.setGoal(turretMotor.getEncoder().getPosition());

        // Rotation2d turretRotation = new Rotation2d(turretMotor.getEncoder().getPosition()).plus(drivetrain.getSwerveDrive().getPose().getRotation());
        
        turretPose = drivetrain.getSwerveDrive().getPose().plus(new Transform2d(-0.3,0.3, getTurretRotation()));
    }

    @Override
    public void periodic(){

        runPID();

        turretRotation = new Rotation2d(turretMotor.getEncoder().getPosition()).plus(drivetrain.getSwerveDrive().getPose().getRotation());

        turretPose = drivetrain.getSwerveDrive().getPose().plus(new Transform2d(-0.3,0.3, getTurretRotation()));
    }

    public void setTurretAngle(Rotation2d rot) {
        double delta1 = rot.getDegrees() - getTurretRotation().getDegrees();
        delta1 = Math.IEEEremainder(delta1, 360);
        turretPID.setGoal(delta1);
    }

    public boolean isTurretAligned(){
        return turretPID.atGoal();
    }

    public Rotation2d getTurretRotation() {
        return turretRotation;
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    private void runPID(){
        // if(Zeroed){
        if (turretMotor.getEncoder().getPosition() < 45 && turretMotor.getEncoder().getPosition() > -45){
            turretMotor.setVoltage(turretPID.calculate(turretMotor.getEncoder().getPosition()));
        } else if (turretMotor.getEncoder().getPosition() > 45 && turretPID.calculate(turretMotor.getEncoder().getPosition())<0){
            turretMotor.setVoltage(turretPID.calculate(turretMotor.getEncoder().getPosition()));
        }else if (turretMotor.getEncoder().getPosition() > -45 && turretPID.calculate(turretMotor.getEncoder().getPosition())>0){
            turretMotor.setVoltage(turretPID.calculate(turretMotor.getEncoder().getPosition()));
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
