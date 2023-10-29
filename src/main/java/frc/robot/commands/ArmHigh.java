package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class ArmHigh extends SequentialCommandGroup{
    
    public ArmHigh(ArmSubsystem arm){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.ARM_HIGH_POSITION), arm)
        );
           
    }
}
