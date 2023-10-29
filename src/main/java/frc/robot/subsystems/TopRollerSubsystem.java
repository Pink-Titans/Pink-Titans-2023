package frc.robot.subsystems;

import frc.robot.Constants;

import frc.robot.utils.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TopRollerSubsystem extends SubsystemBase{

    private CANSparkMax topRollerMotor = new CANSparkMax(Constants.TOP_ROLLER_MOTOR, MotorType.kBrushless);
    
    public void setupMotors(){
      
        topRollerMotor.restoreFactoryDefaults();

        topRollerMotor.setInverted(false);

        topRollerMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_TOP_ROLLER);

        topRollerMotor.setOpenLoopRampRate(0.05);
        
        topRollerMotor.burnFlash();
    }

    public void stop(){
            topRollerMotor.stopMotor();
          }

    public void cubeIntake(double speed){
        topRollerMotor.set(speed);
      }

    public void cubeOuttake(double speed){
        topRollerMotor.set(speed);
      }
        
    public TopRollerSubsystem() {
        setupMotors();
      }
}
