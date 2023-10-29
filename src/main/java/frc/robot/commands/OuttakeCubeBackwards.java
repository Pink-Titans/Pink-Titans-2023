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


public class OuttakeCubeBackwards extends SequentialCommandGroup{
    
    public OuttakeCubeBackwards(IntakeSubsystem intake, ArmSubsystem arm, TopRollerSubsystem roller){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.DOWN_POSITION_BACKWARDS), arm),
            new WaitCommand(0.5),
            new RunCommand(() -> intake.cubeOuttake(Constants.OUTTAKE_SPEED_CUBE), intake),
            new RunCommand(() -> roller.cubeOuttake(Constants.TOP_ROLLER_SPEED_CUBE), roller)
        );
           
    }
}
