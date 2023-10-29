package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class IntakeCube extends SequentialCommandGroup{
    
    IntakeSubsystem intake;
    public IntakeCube(IntakeSubsystem intake){
        addCommands(
            new RunCommand(() -> intake.cubeIntake(Constants.INTAKE_FAST), intake)
        );
           
    }
    
}
