package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX horizontal_turretMotor = new TalonFX(1);
    private final TalonFX vertical_turretMotor = new TalonFX(2);
    private final Canandmag turretEncoder = new Canandmag(2);

    public TurretSubsystem() {
        // Configure the motor to use the encoder for feedback
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.voltageCompSaturation = 12.0;
        horizontal_turretMotor.getConfigurator().apply(configs);
        horizontal_turretMotor.getSensorCollection().setIntegratedSensorPosition(0);
        vertical_turretMotor.getConfigurator().apply(configs);
        vertical_turretMotor.getSensorCollection().setIntegratedSensorPosition(0);
    }

    public void setTurretHorizontalAngle(double angleDegrees) {
        // Convert the desired angle to a duty cycle output
        double dutyCycle = angleDegrees / 360.0; // Assuming 360 degrees corresponds to a full rotation
        turretMotor.set(new DutyCycleOut(dutyCycle));
    }

    public void setTurretVerticalAngle(double angleDegrees) {
        // Convert the desired angle to a duty cycle output
        double dutyCycle = angleDegrees / 360.0; // Assuming 360 degrees corresponds to a full rotation
        vertical_turretMotor.set(new DutyCycleOut(dutyCycle));
    }

    public double getCurrentAngle() {
        // Read the current angle from the encoder
        return turretEncoder.getAngle();
    }
}