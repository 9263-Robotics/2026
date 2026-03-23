package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.OuttakeConstants.*;

public class Outtake extends SubsystemBase {
    private final TalonFX motor = new TalonFX(MOTORCANID1, new CANBus("rio"));
    private final Slot0Configs configs = new Slot0Configs();

    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    private final TalonFX motorfollower = new TalonFX(MOTORCANID2, new CANBus("rio"));

    Outtake() {
        motorfollower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));

        configs.kP = P;
        configs.kI = I;
        configs.kD = D;
        motor.getConfigurator().apply(configs);
    }
    
    public Command spin(double RPS) {
        return runEnd(
            () -> {
                motor.setControl(request.withVelocity(RPS / RATIO));
            },
            () -> {
                motor.setControl(request.withVelocity(0));
                motor.stopMotor(); // just for good measure
            }
        );
    }

    public boolean atSetpoint() {
        return TOLERANCE > Math.abs(motor.getClosedLoopError().getValueAsDouble());
    }
}
