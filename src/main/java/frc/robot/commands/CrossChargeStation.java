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
public class CrossChargeStation extends CommandBase
{

  private final SwerveSubsystem drive;
  private Boolean backwards = false;
  boolean hasRisen = false;
  double startAngle = 360;
  double currentAngle;
  double startX;
  boolean setStartX = false;
  int speedMultiplier = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public CrossChargeStation(SwerveSubsystem drive)
  {
    this.drive = drive;
    addRequirements(drive);
  }

  public CrossChargeStation(SwerveSubsystem drive, Boolean backwards)
  {
    this.drive = drive;
    this.backwards = backwards;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    hasRisen = false;
    startAngle = drive.getPitch().getDegrees();
    speedMultiplier = backwards ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    currentAngle = drive.getPitch().getDegrees() - startAngle;  //FIXME
    System.out.println(hasRisen);
    System.out.println(currentAngle);
    if(currentAngle < Constants.DRIVE_CROSS_RISING_THRESHOLD){
        System.out.println("Print 1");
        hasRisen = true;
    }

    if(!hasRisen || (hasRisen && currentAngle < Constants.DRIVE_CROSS_FALLING_THRESHOLD)){
        System.out.println("Print 2");
        drive.setChassisSpeeds(new ChassisSpeeds(speedMultiplier * Constants.DRIVE_CLIMB_FAST_SPEED, 0, 0));
    }else{
      if(!setStartX){
        startX = drive.getPose().getX();
        System.out.println("Start X: " + startX);
        setStartX = true;
      }
      drive.setChassisSpeeds(new ChassisSpeeds(speedMultiplier * Constants.DRIVE_CLIMB_FAST_SPEED, 0, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
        if(!hasRisen || (hasRisen && currentAngle < Constants.DRIVE_CROSS_FALLING_THRESHOLD)){
            System.out.println("Print 4");
            return false;
        }else{
            if(setStartX && Math.abs(startX - drive.getPose().getX()) > 0.20){
              System.out.println("End X: " + drive.getPose().getX());
              System.out.println("Print 5");
              return true;
            }else{
              return false;
            }
        }  
    }
}