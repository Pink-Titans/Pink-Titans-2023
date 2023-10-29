package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import frc.robot.utils.CANSparkMax;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ONE, MotorType.kBrushed);
    private CANSparkMax intakeMotorFollower = new CANSparkMax(Constants.INTAKE_MOTOR_TWO, MotorType.kBrushed);

    private RelativeEncoder intakeEncoder;

    private SparkMaxPIDController intakePID;

    
    public void setupMotors(){

        SmartDashboard.putNumber("Intake Speed", 0);

      
        intakeMotorFollower.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        intakeMotorFollower.follow(intakeMotor, false);


        intakeMotor.setInverted(false);

        intakeMotor.setIdleMode(IdleMode.kCoast);

        intakeMotor.setSmartCurrentLimit(60);
        intakeMotorFollower.setSmartCurrentLimit(60);

    
        intakeMotor.setOpenLoopRampRate(0.01);
        
        intakeMotor.burnFlash();
        intakeMotorFollower.burnFlash();
    }

    

    public void stop(){
            setHoldingCurrent();
    }
    
    public void stopHoldingCurrent(){
            intakeMotor.set(0);
        }

    public void cubeIntake(double speed){
      intakeMotor.set(1*speed);
      }

    public void setHoldingCurrent(){
      intakeMotor.setVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE);
      }

    public void cubeOuttake(double speed){
      intakeMotor.set(-1*speed);
    }
    


    public IntakeSubsystem() {
        setupMotors();
      }
}
