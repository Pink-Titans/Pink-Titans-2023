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


public class ShootL3Backwards extends SequentialCommandGroup{
    
    public ShootL3Backwards(IntakeSubsystem intake, ArmSubsystem arm, TopRollerSubsystem roller){
        addCommands(
            new GoToAngle(arm, -20),
            new RunCommand(() -> intake.cubeShoot(Constants.SHOOT_L3_BACKWARDS_SPEED), intake)
        );
           
    }

}
