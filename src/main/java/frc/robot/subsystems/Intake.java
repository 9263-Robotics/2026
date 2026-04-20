package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;



import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(14);

  

  private final SparkMax pidMotor1 = new SparkMax(12, MotorType.kBrushless);
  private final SparkMax pidMotor2 = new SparkMax(13, MotorType.kBrushless);  

  private final RelativeEncoder encoder = pidMotor1.getEncoder();


  
  private final ProfiledPIDController intakePID = new ProfiledPIDController(0.035, 0.0, 0.0, new Constraints(25,60));
  // private final PIDController intakePID = new PIDController(0.035, 0.0, 0.0);
  private double setpoint = 0.0;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    encoder.setPosition(0);

    Shuffleboard.getTab(getName()).addDouble("intake setpoint", () -> setpoint);
    Shuffleboard.getTab(getName()).addDouble("intake encoder", () -> getPosition());
    Shuffleboard.getTab(getName()).add(intakePID);


    intakePID.setTolerance(1);
    // pid.setSetpoint(53);
  }

  public void startIntake(){
      intakeMotor.set(-0.7); //full speed
  }

  public void stopIntake(){
      intakeMotor.set(0.0); //stop
  }

  public double getPosition() {
    return encoder.getPosition();  
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()){
        intakePID.setGoal(getPosition());
    }
    intakePID.getSetpoint();
    double output = intakePID.calculate(getPosition(), intakePID.getGoal()); // takes the current position and the desired setpoint and calculates the output
    output = MathUtil.clamp(output, -0.5, 0.5); // limits the output to be between -0.5 and 0.5
    pidMotor1.set(output); // sets the motor output to the calculated value
    pidMotor2.set(-output); //negative cuz facing other way
  } //positive for counter clockwise, negative for clockwise

  public Command setSetpoint(double point){
    return run(() -> {
      intakePID.setGoal(point);//this is the method that will be called to change the setpoint of the PID controller
    }); 
  }

  public Command IntakeDown() {
    return runOnce (() -> {
      intakePID.setConstraints(new Constraints(25,60));
      intakePID.setGoal(IntakeConstants.DownPos);
    });
  }

  public Command IntakeUp() {
    return runOnce(() -> {
      intakePID.setConstraints(new Constraints(25,60));
      intakePID.setGoal(IntakeConstants.UpPos);
    });
  }

  public Command StartIntake() {
    return runOnce(() -> {
      startIntake();
    });
  }

  public Command StopIntake() {
    return runOnce(() -> {
      stopIntake();
    });
  }

  public Command IntakeShootIn() {
    return runOnce(() -> {
      intakePID.setConstraints(new Constraints(5,60));
      intakePID.setGoal(IntakeConstants.MiddlePos);
      intakeMotor.set(-0.4);
    });
  }

  public Command IntakeOut() {
    return runEnd(() -> {
      intakeMotor.set(0.6);
    }, () -> {
      intakeMotor.set(0);
    });
  }

  public Command IntakeShootInConstant() {
    return new SequentialCommandGroup(
      IntakeShootIn(),
      new WaitCommand(2.5),
      IntakeDown(),
      new WaitCommand(1)
    ).repeatedly().finallyDo(() -> stopIntake());
  }

  public Command IntakeMiddle() {
    return runOnce(() -> {
      intakePID.setConstraints(new Constraints(25,60));
      intakePID.setGoal(IntakeConstants.MiddlePos);
    });
  }

  public Command IntakeShake() {
    return new SequentialCommandGroup(
      IntakeMiddle(),
      new WaitCommand(0.4),
      IntakeDown(),
      new WaitCommand(0.4)
    ).repeatedly();
  }

  public Command DrumShooterIntake() {
    return runEnd(() -> {
      intakeMotor.set(1);
    }, () -> {
      intakeMotor.set(0);
    });
  }
  
  public Command runIntakeMotor() {
    return runEnd(() -> {
        intakeMotor.set(-0.5);
    }, () -> {
        intakeMotor.set(0);
    });
  }

  public Command runIntakeWithPID() {
    return new SequentialCommandGroup(
      IntakeDown(),
      new WaitUntilCommand(() -> atSetpoint()),
      runIntakeMotor()
    );
  }

  public boolean atSetpoint(){
    return intakePID.atGoal();
  }


}