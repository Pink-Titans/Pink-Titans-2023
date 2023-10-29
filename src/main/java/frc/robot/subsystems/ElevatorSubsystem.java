package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.utils.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase{

    private CANSparkMax elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
    private CANSparkMax elevatorMotorFollower = new CANSparkMax(Constants.ELEVATOR_MOTOR_TWO, MotorType.kBrushless);

    private RelativeEncoder elevatorEncoder;

    private SparkMaxPIDController elevatorPID;
    SimpleMotorFeedforward elevatorFF = new SimpleMotorFeedforward(Constants.ELEVATOR_KS, Constants.ELEVATOR_KV,
    Constants.ELEVATOR_KA);

    public void setupMotors(){



      elevatorMotor.restoreFactoryDefaults();
      elevatorMotorFollower.restoreFactoryDefaults();


      elevatorMotor.setInverted(false);

      elevatorMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_ELEVATOR);
      elevatorMotorFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_ELEVATOR);

      elevatorMotorFollower.follow(elevatorMotor, false);

        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorEncoder.setPositionConversionFactor(Constants.ELEVATOR_MULTIPLIER);

        elevatorPID = elevatorMotor.getPIDController();
        elevatorEncoder.setVelocityConversionFactor(Constants.ELEVATOR_MULTIPLIER /Constants.SECONDS_PER_MINUTE);

        elevatorPID.setP(Constants.ELEVATOR_PID0_P, 0);
        elevatorPID.setI(Constants.ELEVATOR_PID0_I, 0);
        elevatorPID.setD(Constants.ELEVATOR_PID0_D, 0);
        elevatorPID.setIZone(0, 0);
        elevatorPID.setFF(Constants.ELEVATOR_PID0_F, 0);
        elevatorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

        resetElevatorPosition();
        
        elevatorMotor.burnFlash();
        elevatorMotorFollower.burnFlash();
    }

    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
      }
    
      public double getElevatorVelocity() {
        return elevatorEncoder.getVelocity();
      }
    
      public void resetElevatorPosition() {
        elevatorEncoder.setPosition(0);
      }
    
      // change the angle !!!!
      public void stopElevator() {
        gotoElevatorPosition(elevatorEncoder.getPosition());
      }
    
      public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
    
      }
    
    
      public void gotoElevatorPosition(double angle) {
        elevatorPID.setReference(angle, ControlType.kPosition, Constants.ELEVATOR_PID_SLOT_POSITION);
      }

       public void useOutputPosition(double output, TrapezoidProfile.State setpoint) {

        double angle = elevatorEncoder.getPosition();
    
        double arbFF = elevatorFF.calculate(Units.degreesToRadians(angle), Units.degreesToRadians(setpoint.velocity));
        elevatorPID.setReference(setpoint.position, ControlType.kPosition, Constants.ELEVATOR_PID_SLOT_POSITION, arbFF,
            SparkMaxPIDController.ArbFFUnits.kVoltage);
      }

      public ElevatorSubsystem() {
        setupMotors();
      }

      public void periodic(){
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
      }
    }
