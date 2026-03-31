package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import static frc.robot.Constants.HoodConstants.*;

import java.util.function.DoubleSupplier;

public class Hood extends SubsystemBase {
    public final SparkMax motor = new SparkMax(17, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    private final double[] rots = {0.0, -11, -26};
    private int i = 0;

    public Hood() {
        config.closedLoop.pid(0.005, 0, 0);
        config.closedLoop.outputRange(-1, 1); // PLACEHOLDER

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        Shuffleboard.getTab(getName()).addDouble("Hood angle", () -> motor.getEncoder().getPosition());
        Shuffleboard.getTab(getName()).addDouble("Hood setpoint", () -> getSetpoint());


        // config.encoder.positionConversionFactor(360);
    }

    public Command setHoodAngle(double angle) {
        return runOnce(
            () -> {
                // if (angle >= MINANGLEROT && angle <= MAXANGLEROT) shouldnt be necessary with only three setpoints
                controller.setSetpoint(angle, ControlType.kPosition);
                
            }
        );
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Hood angle", motor.getEncoder().getPosition());
        // SmartDashboard.putNumber("Hood setpoint", getSetpoint());
        setHoodAngle(rots[i]);
    }

    public Command setHoodAngle(DoubleSupplier radianSupplier){
        return defer(() -> {
            return setHoodAngle(radianSupplier.getAsDouble());
        });
    }

    public double getSetpoint(){
        return controller.getSetpoint();
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