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

public class ArmSubsystem extends SubsystemBase{

    private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
    private RelativeEncoder armEncoder;

    private SparkMaxPIDController armPID;
    SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(Constants.ARM_KS, Constants.ARM_KV,
    Constants.ARM_KA);

    public void setupMotors(){

        armMotor.restoreFactoryDefaults();

        armMotor.setInverted(false);

        armMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_ARM);

        armEncoder = armMotor.getEncoder();

        armEncoder.setPositionConversionFactor(Constants.DEGREES_PER_ROTATION / Constants.ARM_GEAR_RATIO);

        armPID = armMotor.getPIDController();
        armEncoder.setVelocityConversionFactor(Constants.DEGREES_PER_ROTATION / ((double)Constants.ARM_GEAR_RATIO * Constants.SECONDS_PER_MINUTE));

        armPID.setP(Constants.ARM_PID0_P, 0);
        armPID.setI(Constants.ARM_PID0_I, 0);
        armPID.setD(Constants.ARM_PID0_D, 0);
        armPID.setIZone(0, 0);
        armPID.setFF(Constants.ARM_PID0_F, 0);
        armPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

        resetArmAngle();
        
        armMotor.burnFlash();
    }

    public double getArmAngle() {
        return armEncoder.getPosition();
      }
    
      public double getArmVelocity() {
        return armEncoder.getVelocity();
      }
    
      public void resetArmAngle() {
        armEncoder.setPosition(0);
      }
    
      // change the angle !!!!
      public void stopArm() {
        gotoArmAngle(armEncoder.getPosition());
      }
    
      public void setArmSpeed(double speed) {
        armMotor.set(speed);
    
      }
    
    
      public void gotoArmAngle(double angle) {
        armPID.setReference(angle, ControlType.kPosition, Constants.ARM_PID_SLOT_POSITION);
      }

       public void useOutputPosition(double output, TrapezoidProfile.State setpoint) {

        double angle = armEncoder.getPosition();
    
        double arbFF = armFF.calculate(Units.degreesToRadians(angle), Units.degreesToRadians(setpoint.velocity));
        armPID.setReference(setpoint.position, ControlType.kPosition, Constants.ARM_PID_SLOT_POSITION, arbFF,
            SparkMaxPIDController.ArbFFUnits.kVoltage);
      }

      public ArmSubsystem() {
        setupMotors();
      }

      public void periodic(){
        SmartDashboard.putNumber("Arm Angle", getArmAngle());
      }
    }
