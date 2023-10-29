// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ArmGoDown;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ArmLow;
import frc.robot.commands.ArmMid;
import frc.robot.commands.ElevatorHigh;
import frc.robot.commands.ElevatorLow;
import frc.robot.commands.ElevatorMid;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToAngle;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.OuttakeCube;
import frc.robot.commands.StopIntake;
import frc.robot.commands.ShootCube;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.MoveArmTrapezoid;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AlignWithDirection;

import java.io.File;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.autos.DriveStraight;
import frc.robot.autos.DoNothing;
import frc.robot.autos.Balance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  IntakeSubsystem intake = new IntakeSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  ElevatorSubsystem elevator = new ElevatorSubsystem();
  CommandBase driveStraight = DriveStraight.auto(swerve);
  CommandBase doNothing = DoNothing.auto();
  CommandBase balance = Balance.auto(swerve);

 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);

  DigitalInput m_limitSwitch = new DigitalInput(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    TeleopDrive driveCommand = new TeleopDrive(
    swerve,
    () -> aboveDeadband(-driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
    () -> aboveDeadband(-driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
    () -> aboveDeadband(driverController.getRightX()), () -> true, true, false);

  swerve.setDefaultCommand(driveCommand);

  intake.setDefaultCommand(new StopIntake(intake));
  //arm.setDefaultCommand(new ArmHigh(arm));


  m_chooser.setDefaultOption("1 - Drive Straight Auto", driveStraight);
  m_chooser.addOption("2 - Do Nothing", doNothing);
  m_chooser.addOption("3 - Balance", balance);
  SmartDashboard.putData(m_chooser);

  }
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.back().onTrue((new InstantCommand(swerve::zeroGyro)));


      /*   driverController.start().and(driverController.povDown()).whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY()),
                () -> -driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY()),
                () -> 180
                )
        );

        driverController.start().and(driverController.povLeft()).whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY()),
                () -> -driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY()),
                () -> 90
                )
        );

        driverController.start().and(driverController.povRight()).whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY()),
                () -> -driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY()),
                () -> -90
                )
        );
*/
    driverController.x().whileTrue(new RepeatCommand(new InstantCommand(swerve::lock, swerve)));
/*
    driverController.a()
    .onTrue(new ShootL2(intake, arm, roller)).whileFalse(new InstantCommand(() -> arm.gotoArmAngle(Constants.REST_POSITION), arm));

    driverController.b()
    .onTrue(new ShootL3(intake, arm, roller)).whileFalse(new InstantCommand(() -> arm.gotoArmAngle(Constants.REST_POSITION), arm));

    driverController.a().and(driverController.rightBumper())
    .onTrue(new ShootL2Backwards(intake, arm, roller)).whileFalse(new InstantCommand(() -> arm.gotoArmAngle(Constants.REST_POSITION), arm));

    driverController.b().and(driverController.rightBumper())
    .onTrue(new ShootL3Backwards(intake, arm, roller)).whileFalse(new InstantCommand(() -> arm.gotoArmAngle(Constants.REST_POSITION), arm));

    driverController.rightBumper().whileTrue(new IntakeCube(intake, arm, roller)).whileFalse(new InstantCommand(() -> arm.gotoArmAngle(Constants.REST_POSITION), arm));

    driverController.rightTrigger().and(driverController.rightBumper()).whileTrue(new IntakeCubeBackwards(intake, arm, roller));

    driverController.leftTrigger().whileTrue(new OuttakeCube(intake, arm, roller));

    driverController.leftTrigger().and(driverController.rightBumper()).whileTrue(new OuttakeCubeBackwards(intake, arm, roller));

    driverController.y()
    .onTrue(new ShootL3(intake, arm, roller)).whileFalse(new InstantCommand(() -> arm.gotoArmAngle(Constants.REST_POSITION), arm));*/

    //driverController.a().whileTrue(new IntakeSpin());
    //driverController.b().whileTrue(new ArmGoDown(intake, arm, roller));
    //driverController.x().whileTrue(new RollerSpin(roller));
    //driverController.x().whileTrue(new IntakeCube(intake, arm));

    driverController.rightBumper()
    .whileTrue(new IntakeCube(intake));

    driverController.leftBumper()
    .whileTrue(new OuttakeCube(intake));

    driverController.leftTrigger().whileTrue(new ShootCube(intake));


    operatorController.y()
    .whileTrue(new ArmHigh(arm));

    operatorController.x()
    .whileTrue(new ArmMid(arm));


    operatorController.b()
    .whileTrue(new ArmMid(arm));


    operatorController.a()
    .whileTrue(new ArmLow(arm));


    operatorController.povUp()
    .whileTrue(new ElevatorHigh(elevator));

    operatorController.povLeft()
    .whileTrue(new ElevatorMid(elevator));

    operatorController.povRight()
    .whileTrue(new ElevatorMid(elevator));

    operatorController.povDown()
    .whileTrue(new ElevatorLow(elevator));


    //state.whileActiveContinuous(new Rumble(driverGamepad, true, true)).whenInactive(new Rumble(driverGamepad, true, false));
    //m_limitSwitch.whileActiveContinuous(new Rumble(driverGamepad, false, true)).whenInactive(new Rumble(driverGamepad, false, false));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public double inputScale(double x, double y) {
   // x = (x*0.8)+0.2;
    //y = (y*0.8)+0.2;
    
    double xAbs = Math.abs(x);
    double yAbs = Math.abs(y);
    double angle = Math.atan2(yAbs, xAbs);
    double result = yAbs > xAbs ? Math.sin(angle) : Math.cos(angle);
    return result;
}

   public double aboveDeadband(double input) {
    return (Math.abs(input) > Constants.DEFAULT_DEADBAND) ? ((input*0.8)+((Math.signum(input)*0.2))) : 0;
}

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
