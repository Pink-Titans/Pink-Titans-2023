package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightSubsystem extends SubsystemBase{

    CANdle candle = new CANdle(10);

    CANdleConfiguration config = new CANdleConfiguration();
    
    public void setupLights(){
        config.stripType = LEDStripType.RGB; 
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    public void blueColour(){
        candle.clearAnimation(0);
        candle.setLEDs(0, 255, 0);
    }

    public void redColour(){
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 0);
    }

    public void greenColour(){
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 255);
    }

    public void whiteColour(){
        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 255);
    }

    public void requestCube(){
        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 0);
    }

    public LightSubsystem(){
        setupLights();
    }
    public void setLights(){
        if (Constants.currentAlliance != Alliance.Invalid) {
             if(Constants.currentAlliance == Alliance.Blue){
                blueColour();
             }else{
                redColour();
             }
          }else{
            whiteColour();
          }
    }
}
