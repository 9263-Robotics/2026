package frc.robot.subsystems;

// import static frc.robot.Constants.HoodConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    public final SparkMax motor = new SparkMax(17, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    private double DesiredHoodAngle = 0;

    private final double[] rots = {0.0, -11, -26};
    private int i = 0;

    public Hood() {
        config.closedLoop.pid(0.13
        , 0, 0);
        config.closedLoop.outputRange(-1, 1); // PLACEHOLDER

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        Shuffleboard.getTab(getName()).addDouble("Hood angle", () -> motor.getEncoder().getPosition());
        Shuffleboard.getTab(getName()).addDouble("Hood setpoint", () -> getSetpoint());
        Shuffleboard.getTab(getName()).addDouble("Hood output", () -> motor.getAppliedOutput());

        Shuffleboard.getTab(getName()).addBoolean("Trench Good", () -> trenchGood());

        SmartDashboard.putNumber("Hood Angle", 0);


        // config.encoder.positionConversionFactor(360);
    }
public boolean trenchGood() {
    return motor.getEncoder().getPosition() > -2;
    
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
        if (DriverStation.isDisabled()){
            DesiredHoodAngle = motor.getEncoder().getPosition();
            controller.setSetpoint(DesiredHoodAngle, ControlType.kPosition);
        } else {
            // controller.setSetpoint(rots[i], ControlType.kPosition);
            // controller.setSetpoint(, ControlType.kPosition);

            controller.setSetpoint(DesiredHoodAngle, ControlType.kPosition);
            // controller.setSetpoint(MathUtil.clamp(SmartDashboard.getNumber("Hood Angle", 0), -26, 0),ControlType.kPosition);

        }
        
        
    }

    public Command setHoodAngle(DoubleSupplier radianSupplier){
        return defer(() -> {
            return setHoodAngle(radianSupplier.getAsDouble());
        });
    }

    public void setHoodAngleFunc(double hoodAngle) {
        if(hoodAngle > -26 && hoodAngle < 0) {
            DesiredHoodAngle = hoodAngle;
            controller.setSetpoint(hoodAngle, ControlType.kPosition);
        } else if(hoodAngle < -26) {
            DesiredHoodAngle = -26;
            controller.setSetpoint(-26, ControlType.kPosition);
        } else {
            DesiredHoodAngle = 0;
            controller.setSetpoint(0, ControlType.kPosition);
        }
    }

    public double getSetpoint(){
        return controller.getSetpoint();
    }

    public Command HoodDown (){
        return runEnd(
            () -> {
                i = 0;
            },
            () -> {
                i = 0;
            }
        );
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
