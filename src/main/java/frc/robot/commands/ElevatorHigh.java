package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;


public class ElevatorHigh extends SequentialCommandGroup{
    
    public ElevatorHigh(ElevatorSubsystem elevator){
        addCommands(
            new RunCommand(() -> elevator.gotoElevatorPosition(Constants.ELEVATOR_HIGH_POSITION), elevator)
        );
           
    }
}
