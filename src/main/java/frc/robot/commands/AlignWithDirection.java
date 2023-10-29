package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import swervelib.SwerveController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;



/** A command that will turn the robot to the specified angle. */
public class AlignWithDirection extends CommandBase{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private DoubleSupplier angle;
  private DoubleSupplier headingX;
  private DoubleSupplier headingY;

  private TrapezoidProfile.Constraints m_constraints;
private ProfiledPIDController rotationController;
private SimpleMotorFeedforward arbFF;


  
  private final SwerveController controller;
  private double targetAngle;

    //DriveSubsystem drive;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AlignWithDirection(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier angle) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.angle = angle;
        this.controller = swerve.getSwerveController();
        addRequirements(swerve);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
       double currentAngle = swerve.getHeading().getDegrees();
       if(Math.abs(currentAngle) < 90){
        targetAngle = 0;
       }else{
        targetAngle = 180;
       }
      System.out.println("Align With Direction - Start");
      arbFF = new SimpleMotorFeedforward(0.15, 0.75*controller.config.maxAngularVelocity/90);

    m_constraints =   new TrapezoidProfile.Constraints(100*controller.config.maxAngularVelocity, 100*controller.config.maxAngularVelocity);

    
    rotationController =   new ProfiledPIDController(0.01, 0, 0, m_constraints);


    rotationController.reset(targetAngle);


      rotationController.enableContinuousInput(-180, 180);
      rotationController.setTolerance(0.5);
     // rotationController.setGoal(targetAngle);  
    }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute(){
    //rotationController.setP(SmartDashboard.getNumber("rotP", 0));
   // rotationController.setD(SmartDashboard.getNumber("rotD", 0));

      double rotationRate = 0;
      SmartDashboard.putNumber("RotTargetAngle", targetAngle);
      SmartDashboard.putNumber("RotCurrent", swerve.getHeading().getDegrees());
      rotationRate = rotationController.calculate(swerve.getHeading().getDegrees(), targetAngle);
      SmartDashboard.putNumber("RotPIDOutput", rotationRate);
     // SmartDashboard.putNumber("RotSetPoint", rotationController.getSetpoint().position);

      double arbFFValue = arbFF.calculate(rotationController.getPositionError());
      SmartDashboard.putNumber("RotError", rotationController.getPositionError());
     // double setPoint = rotationController.getSetpoint().position;
      SmartDashboard.putNumber("RotArbFF", arbFFValue);
      SmartDashboard.putBoolean("RotAtSetPoint", rotationController.atSetpoint());
     // SmartDashboard.putBoolean("RotAtGoal", rotationController.atGoal());

      rotationRate += arbFFValue;
      if(rotationController.atSetpoint()){
        rotationRate=0;
      }

     swerve.drive(new Translation2d(Math.pow(vX.getAsDouble(), 3)*controller.config.maxSpeed, Math.pow(vY.getAsDouble(), 3)*controller.config.maxSpeed), MathUtil.clamp(rotationRate, -controller.config.maxAngularVelocity, controller.config.maxAngularVelocity), true, false);
     SmartDashboard.putNumber("vX", vX.getAsDouble());
     SmartDashboard.putNumber("vY", vY.getAsDouble());
 
   }

  @Override
  public boolean isFinished() {
    //return rotationController.atSetpoint();
    return false;
  }



  @Override
  public void end(boolean interrupted) {
      //swerve.drive(new Translation2d(0, 0), 0, false, false);
      System.out.println("Align With Direction - End");
      System.out.println("Align With Direction - Interrupted - " + interrupted);
  }
}
