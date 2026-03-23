package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HoodConstants.*;

import java.util.function.DoubleSupplier;

public class Hood extends SubsystemBase {
    private final SparkMax motor = new SparkMax(MOTORCANID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    public Hood() {
        config.closedLoop.pid(P, I, D);
        // config.encoder.positionConversionFactor(360);
    }

    public Command setHoodAngle(double radians) {
        return runOnce(
            () -> {
                if (radians >= MINANGLERAD && radians <= MAXANGLERAD)
                    controller.setSetpoint(radians, ControlType.kPosition);
            }
        );
    }

    public Command setHoodAngle(DoubleSupplier radianSupplier){
        return defer(() -> {
            return setHoodAngle(radianSupplier.getAsDouble());
        });
    }
}
