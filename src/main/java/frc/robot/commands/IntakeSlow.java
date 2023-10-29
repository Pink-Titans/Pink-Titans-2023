package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.Constants;


public class IntakeSlow extends SequentialCommandGroup{
    
    public IntakeSlow(IntakeSubsystem intake, double speed){
        addCommands(
            new RunCommand(() -> intake.cubeIntake(0.2), intake)
           // new RunCommand(() -> roller.cubeIntake(Constants.TOP_ROLLER_SPEED_CUBE), roller)
        );
           
    }
}