package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.commands.CrossChargeStation;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.Reset;
import frc.robot.commands.ShootL3;
import frc.robot.commands.ShootL3Backwards;

import java.util.HashMap;
import java.util.List;

public final class L3MobilityBump
{

  /**
   * April Tag field layout.
   */

  private L3MobilityBump()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake, TopRollerSubsystem roller)
  {
    PathPlannerTrajectory driveStraight;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveStraight = PathPlanner.generatePath(
        new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
// position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(4.5, 0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
// position, heading(direction of travel), holonomic rotation
                                        );

      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
   swerve.postTrajectory(driveStraight);
    return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new ShootL3Backwards(intake, arm, roller)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new Reset(swerve, arm, intake, roller)
      ),
        new FollowTrajectory(swerve, driveStraight, true)
         ));
  }

 
}
