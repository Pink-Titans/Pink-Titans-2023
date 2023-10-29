package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends SequentialCommandGroup {
    
    public StopIntake(IntakeSubsystem intake){
        
        addCommands(
            new RunCommand(() -> intake.stop(), intake)
        );
    }
}
