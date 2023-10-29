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
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.Constants;

public class IntakeCube extends SequentialCommandGroup{
    
    ArmSubsystem arm;
    IntakeSubsystem intake;
    TopRollerSubsystem roller;
    public IntakeCube(IntakeSubsystem intake, ArmSubsystem arm, TopRollerSubsystem roller){
        addCommands(
            new ParallelCommandGroup(
            new GoToAngle(arm, Constants.DOWN_POSITION),
            new IntakeSpin(intake, Constants.INTAKE_SPEED_CUBE),
            new InstantCommand(() -> roller.cubeIntake(Constants.TOP_ROLLER_SPEED_CUBE), roller))
        );
           
    }
    
    public void end(){
        
        new GoToAngle(arm, Constants.DOWN_POSITION);

        
    }
}
