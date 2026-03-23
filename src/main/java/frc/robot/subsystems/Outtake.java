package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.OuttakeConstants.*;

// suboptimal code (i.e. not done)

public class Outtake extends SubsystemBase {
    // two krakens, same direction
    // must make flywheel maintain velocity
    // provide a boolean method which returns whether the flywheel velocity is good.
    // gear ratio: 1.2
    private final TalonFX motor1 = new TalonFX(MOTORID1, new CANBus("rio"));
    private final TalonFX motor2 = new TalonFX(MOTORID2, new CANBus("rio"));

    private final PIDController motorFeedback1 = new PIDController(P, I, D);
    private final PIDController motorFeedback2 = new PIDController(P, I, D);

    Outtake() {
        setDefaultCommand(
            runOnce(
                () -> {
                    motor1.disable();
                    motor2.disable();
                }
            )
        );
    }

    private double flywheeltomotorRPS(double flywheel) {
        return flywheel / RATIO;
    }

    private double getVelocity(TalonFX motor) {
        return motor.getVelocity().getValueAsDouble();
    }

    public boolean velocityReady(double flywheelRPS) {
        if (getVelocity(motor1) >= flywheelRPS - THRESHOLD && 
            getVelocity(motor1) <= flywheelRPS + THRESHOLD &&
            getVelocity(motor1) >= getVelocity(motor2) - THRESHOLD && 
            getVelocity(motor1) <= getVelocity(motor2) + THRESHOLD)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public Command constantVelocity(double flywheelRPSsetpoint) {
        return run(
            () -> {
                double setpoint = flywheeltomotorRPS(flywheelRPSsetpoint);
                double RPS1 = getVelocity(motor1);
                double RPS2 = getVelocity(motor1);
                // SmartDashboard.putNumber("Outtake RPS 1", RPS1);
                motor1.set(motorFeedback1.calculate(RPS1, setpoint));
                motor2.set(motorFeedback2.calculate(RPS2, setpoint));
            }
        );
    }
}
