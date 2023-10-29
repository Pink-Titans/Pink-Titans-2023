package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

public class SetAllianceColor extends SequentialCommandGroup {
    
    public SetAllianceColor(LightSubsystem light){
        
        addCommands(
            new RunCommand(() -> light.setLights(), light)
        );
    }
}
