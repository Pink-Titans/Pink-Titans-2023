package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class MoveArmTrapezoid extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

   private final ArmSubsystem arm;
   private final double position;
 
  public MoveArmTrapezoid(double position, ArmSubsystem arm) {
    super(
        new ProfiledPIDController(
            Constants.ARM_PID_PROFILED_P,
            Constants.ARM_PID_PROFILED_I,
            Constants.ARM_PID_PROFILED_D,
            new TrapezoidProfile.Constraints(
                Constants.ARM_MAX_SPEED,
                Constants.ARM_MAX_ACCELERATION)),
        // Close loop on heading
        arm::getArmAngle,
        // Set reference to target
        position,
        // Pipe output to turn robot
        (output, setpoint) -> arm.useOutputPosition(output, setpoint),
        // Require the drive
        arm);

    // Set the controller to be continuous (because it is an angle controller)
   // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.ARM_POSITION_TOLERANCE);

    this.position = position;
    this.arm = arm;
  }
/*
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
*/
  @Override
  public void end(boolean interrupted){
    
  }
}