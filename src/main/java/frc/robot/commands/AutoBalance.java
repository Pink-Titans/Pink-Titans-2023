package frc.robot.commands;

import frc.robot.Constants;
import lib.navx.AHRS;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lib.navx.AHRSProtocol.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoBalance extends CommandBase
{

  private final SwerveSubsystem drive;
  private Boolean backwards = false;
  boolean hasRisen = false;
  boolean hasLeveled = false;
  double startAngle;
  double currentAngle;
  int speedMultiplier = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public AutoBalance(SwerveSubsystem drive)
  {
    this.drive = drive;
    addRequirements(drive);
  }

  public AutoBalance(SwerveSubsystem drive, Boolean backwards)
  {
    this.drive = drive;
    this.backwards = backwards;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    hasRisen = false;
    hasLeveled = false;
    startAngle = 0; //drive.getPitch().getDegrees();
    speedMultiplier = backwards ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    currentAngle = drive.getPitch().getDegrees() - startAngle;  //FIXME
    System.out.println(hasRisen);
    System.out.println(currentAngle);
    if(Math.abs(currentAngle) > Constants.DRIVE_CLIMB_RISING_THRESHOLD){
        System.out.println("Print 1");
        hasRisen = true;
    }

    if(!hasLeveled && (!hasRisen || (hasRisen && Math.abs(currentAngle) > Constants.DRIVE_CLIMB_FALLING_THRESHOLD))){
        System.out.println("Print 2");
        drive.setChassisSpeeds(new ChassisSpeeds(speedMultiplier * Constants.DRIVE_CLIMB_RISING_SPEED, 0, 0));
    }else if(Math.abs(currentAngle) > Constants.DRIVE_CLIMB_LEVEL_THRESHOLD){
        System.out.println("Print 3");
        double speed = speedMultiplier*0.4;
        System.out.println(speed);
        drive.setChassisSpeeds(new ChassisSpeeds(speed, 0, 0));
        hasLeveled = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    drive.lock();
    System.out.println("Print 6");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
        if(hasLeveled && (hasRisen && Math.abs(currentAngle) < Constants.DRIVE_CLIMB_LEVEL_THRESHOLD)){
            System.out.println("Print 4");
            return true;
        }else{
            System.out.println("Print 5");
            return false;
        }  
    }
}