package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HoodConstants.*;

import java.util.function.DoubleSupplier;

public class Hood extends SubsystemBase {
    public final SparkMax motor = new SparkMax(17, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    private final double[] rots = {0.0, 0.0, 0.0};
    private int i = 0;

    public Hood() {
        config.closedLoop.pid(0.0, 0.0, 0.0);
        config.closedLoop.outputRange(0, 1); // PLACEHOLDER
        // config.encoder.positionConversionFactor(360);
    }

    public Command setHoodAngle(double angle) {
        return runOnce(
            () -> {
                double truAngle = (angle / 360) / (2000 / 7);
                if (truAngle >= 0.0 && truAngle <= 71.0)
                controller.setSetpoint(truAngle, ControlType.kPosition);
                
            }
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood angle", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood setpoint", getSetpoint());
        setHoodAngle(rots[i]);
    }

    // public Command setHoodAngle(DoubleSupplier radianSupplier){
    //     return defer(() -> {
    //         return setHoodAngle(radianSupplier.getAsDouble());
    //     });
    // }

    public double getSetpoint(){
        return controller.getSetpoint() * 360;
    }

    public Command iterateRot() { 
        return runOnce(
            () -> {
                i++; 
                i %= rots.length; 
            }
        );
    }
}
