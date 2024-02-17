// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private Joystick driverJoystick = new Joystick(OIConstants.DriveJoystick);
  private Joystick driverTurn = new Joystick(OIConstants.TurnJoystick);
  private Joystick operatorJoystick = new Joystick(OIConstants.operatorJoystick);

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooterSubsystem.feedNotes(), shooterSubsystem));
    
    
    ShooterCmd shooterCmd = new ShooterCmd(shooterSubsystem,
    () -> driverJoystick.getRawAxis(3),
    () -> driverJoystick.getRawButton(1),
    () -> driverTurn.getRawButton(1),
    () -> driverTurn.getRawButton(2),
    () -> driverTurn.getRawAxis(3)
    );


    IntakeCmd intakeCmd = new IntakeCmd(intakeSubsystem,
    () -> driverTurn.getRawButton(11),
    () -> driverTurn.getRawButton(16));



    ClimberCmd climberCmd = new ClimberCmd(climberSubsystem,
    () -> driverTurn.getRawButton(7),
    () -> driverTurn.getRawButton(8));


    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverJoystick.getRawAxis(OIConstants.TranslationY),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverJoystick.getRawAxis(OIConstants.TranslationX),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverTurn.getRawAxis(OIConstants.Rotation),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   () -> POVAway(),
                                                                   () -> POVTowards(),
                                                                   () -> POVLeft(),
                                                                   () -> POVRight());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverJoystick.getRawAxis(OIConstants.TranslationY),
         OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverJoystick.getRawAxis(OIConstants.TranslationX),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> driverTurn.getRawAxis(0));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-driverJoystick.getRawAxis(OIConstants.TranslationY),
         OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverJoystick.getRawAxis(OIConstants.TranslationX),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> driverTurn.getRawAxis(0));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocity);

    shooterSubsystem.setDefaultCommand(shooterCmd);
    intakeSubsystem.setDefaultCommand(intakeCmd);
    climberSubsystem.setDefaultCommand(climberCmd);
  }

  
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverJoystick, 2).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverJoystick,
                       3).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(14.6, 5), Rotation2d.fromDegrees(0)))
                              ));
                              
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("1+1CROut", true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public boolean POVAway(){
    return driverTurn.getPOV() == 0;
  }

  public boolean POVLeft(){
    return driverTurn.getPOV() == 270;
  }

  public boolean POVRight(){
    return driverTurn.getPOV() == 90;
  }

  public boolean POVTowards(){
    return driverTurn.getPOV() == 180;
  }
}
