package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

public class GoToAngle extends CommandBase {
    
    final ArmSubsystem arm;
    final Double angle;

    public GoToAngle(ArmSubsystem arm, double angle)
  {
    this.arm = arm;
    this.angle = angle;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    arm.gotoArmAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if(Math.abs(arm.getArmAngle() - angle) <10 && Math.abs(arm.getArmVelocity()) < 1500){
      return true;
    }else {
      return false;
    }  }
}
