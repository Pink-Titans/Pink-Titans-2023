package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase; 

import frc.robot.subsystems.LightSubsystem;

public class AllianceColour extends CommandBase{

    public boolean isDone = false; 
    public String allianceColor;
    final LightSubsystem light;


    public AllianceColour(LightSubsystem light, String allianceColor){
        this.allianceColor = allianceColor;
        this.light = light;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SendableChooser<Command> alliance = new SendableChooser<>();
        alliance.addOption("Red", null);
        alliance.addOption("Bue", null);
        SmartDashboard.putData(alliance);
        allianceColor = String.valueOf(alliance.getSelected());
        if(allianceColor == "Red"){
            light.redColour();
        }
        else if(allianceColor == "Blue"){
            light.blueColour();
        }
    }

    @Override 
    public boolean isFinished() { 
      return isDone; 
    } 
}
