package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class Reset extends ParallelCommandGroup{
    
    public Reset(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake){
        addCommands(
            new GoToAngle(arm, Constants.REST_POSITION),
            new RunCommand(() -> intake.stop(), intake)
        );
           
    }
}
