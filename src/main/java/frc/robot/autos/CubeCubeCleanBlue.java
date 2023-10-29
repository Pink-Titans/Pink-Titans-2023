package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.OuttakeCube;
import frc.robot.commands.OuttakeCubeBackwards;
import frc.robot.commands.ShootL2;
import frc.robot.commands.ShootL3;
import frc.robot.commands.ShootL2Backwards;
import frc.robot.commands.ShootL3Backwards;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

import java.util.HashMap;
import java.util.List;

import org.w3c.dom.events.MouseEvent;


public final class CubeCubeCleanBlue
{

  /**
   * April Tag field layout.
   */
  private CubeCubeCleanBlue()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake, TopRollerSubsystem roller)
  {
    boolean blue = true;
    double multiplier = blue?-1:1;

    PathPlannerTrajectory driveToCube;
    PathPlannerTrajectory driveToGrid;
    PathPlannerTrajectory driveToCube2;
    PathPlannerTrajectory driveBack;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveToCube = PathPlanner.generatePath(
        new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
        new PathPoint(new Translation2d(-2, multiplier*0.05), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
        new PathPoint(new Translation2d(-4.4, multiplier*-0.2), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*170)),
        new PathPoint(new Translation2d(-4.6, multiplier*-0.2), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*170)),
        new PathPoint(new Translation2d(-4.85, multiplier*-0.20), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*170))
    );
    driveToGrid = PathPlanner.generatePath(
      new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
      true,
      new PathPoint(new Translation2d(-4.85, multiplier*-0.20), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*-170)),
      new PathPoint(new Translation2d(-1.5, multiplier*-0.4), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*-5)),
      new PathPoint(new Translation2d(-0.25, multiplier*-0.9), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*-5))
  );

    driveToCube2 = PathPlanner.generatePath(
      new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
      true,
      new PathPoint(new Translation2d(-0.25, multiplier*-0.9), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
      new PathPoint(new Translation2d(-2.2, multiplier*-0.6), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
      new PathPoint(new Translation2d(-3.2, multiplier*-0.6), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
      new PathPoint(new Translation2d(-4.50, multiplier*-1.50), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-160)),
      new PathPoint(new Translation2d(-4.95, multiplier*-1.72), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-160))
  );



      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
      //swerve.postTrajectory(driveToCube);
      //swerve.postTrajectory(driveToGrid);
      return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(driveToCube.getInitialHolonomicPose())),
      new ParallelRaceGroup(
        new WaitCommand(2.5),
        new ShootL2(intake, arm, roller)
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FollowTrajectory(swerve, driveToCube, true)
        )/* 
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(2),
            new OuttakeCube(intake, arm, roller)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeCube(intake, arm, roller)
          )*/
        //)
      ),
      //new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)), 
      new ParallelCommandGroup(
        new FollowTrajectory(swerve, driveToGrid, false),
        new SequentialCommandGroup(
          new WaitCommand(2)
          /* ParallelRaceGroup(
            new WaitCommand(1.5),
            new OuttakeCube(intake, arm, roller)
          )*/
        )
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FollowTrajectory(swerve, driveToCube2, false)
        )/* ,
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(2),
            new OuttakeCube(intake, arm, roller)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeCube(intake, arm, roller)
          )
        )*/
      ),
      //new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      //new InstantCommand(() -> wrist.gotoWristAngle(Constants.WRIST_HOME_POSITION), intake), 
      //new InstantCommand(() -> intake.setGamePieceType(GamePieces.CONE), intake), 
      new WaitCommand(0.5),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))
    ));
  }

 
}
