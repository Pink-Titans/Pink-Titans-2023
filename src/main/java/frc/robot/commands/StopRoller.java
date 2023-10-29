package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

public class StopRoller extends SequentialCommandGroup {
    
    public StopRoller(TopRollerSubsystem roller){
        
        addCommands(
            new RunCommand(() -> roller.stop(), roller)
        );
    }
}
