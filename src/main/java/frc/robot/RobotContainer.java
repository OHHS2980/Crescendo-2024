// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootCloseCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.autoIntakeCmd;
import frc.robot.commands.feederWheelCmd;
import frc.robot.commands.shootNoteAuto;
import frc.robot.commands.shootPosition;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.sql.Driver;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final CameraSubsystem cameraSubsystem = new CameraSubsystem(drivebase);

  private final Translation2d speakerPose2dBlue = new Translation2d(0.0, 5.5);
  private final Translation2d speakerPose2dRed = new Translation2d(16.54, 5.5);

  private Joystick leftJoystick = new Joystick(OIConstants.DriveJoystick);
  private Joystick rightJoystick = new Joystick(OIConstants.TurnJoystick);
  private XboxController operator = new XboxController(OIConstants.operatorJoystick);

  private Command driveFieldOrientedAngle;
  private Command driveFieldOrientedAnglularVelocity;

  private ShooterCmd shooterCmd;
  private shootPosition shootPosition;

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    //NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooterSubsystem.startShooting(), shooterSubsystem));
    //NamedCommands.registerCommand("StopFeed", new InstantCommand(() -> shooterSubsystem.stopFeed(), shooterSubsystem));
    //NamedCommands.registerCommand("Prep", new shootPosition(() -> drivebase.getPose(), shooterSubsystem));
    //NamedCommands.registerCommand("UnFeed", new InstantCommand(() -> shooterSubsystem.unfeedNotes(), shooterSubsystem));
    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(new autoIntakeCmd(intakeSubsystem), new InstantCommand(() -> shooterSubsystem.intakePos(), shooterSubsystem)));
    //NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));
    //NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> shooterSubsystem.stopShooting(), shooterSubsystem));
    //NamedCommands.registerCommand("ShooterIntake", new InstantCommand(() ->shooterSubsystem.intakePos(), shooterSubsystem));
    NamedCommands.registerCommand("LoadNote", new SequentialCommandGroup(new shootNoteAuto(() -> drivebase.getPose(), shooterSubsystem, feederSubsystem), new WaitCommand(0.15), new InstantCommand(() -> shooterSubsystem.stopShooting(), shooterSubsystem)));
    NamedCommands.registerCommand("CenterNote", new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(() -> feederSubsystem.unfeedNotes(), shooterSubsystem), new WaitCommand(0.025), new InstantCommand(() -> feederSubsystem.stopFeed(), shooterSubsystem))));
    //NamedCommands.registerCommand("CenterNotes", new InstantCommand(() -> shooterSubsystem.stopFeed(), shooterSubsystem));
    
    
    shooterCmd = new ShooterCmd(shooterSubsystem,
    () -> rightJoystick.getRawAxis(3), //shooterSpeed
    () -> rightJoystick.getRawButton(1), //runFlywheel
    () -> leftJoystick.getRawAxis(3), //armAim
    () -> operator.getAButtonPressed()//Amp
    );

    shootPosition = new shootPosition(
      () -> drivebase.getPose(), shooterSubsystem,
      () -> leftJoystick.getRawButton(1), //shoot
      () -> operator.getAButtonPressed(), //amp
      () -> rightJoystick.getRawButton(1), //intake
      () -> leftJoystick.getRawButtonPressed(2), //aim
      () -> operator.getYButtonPressed(), //source
      () -> operator.getXButtonPressed(), //stow
      () -> leftJoystick.getRawAxis(3) //offset
    );

    feederWheelCmd feederWheelCmd = new feederWheelCmd(feederSubsystem,
      () -> rightJoystick.getRawButton(4), //shoot
      () -> rightJoystick.getRawButton(1), //intake
      () -> operator.getYButton(), //center
      () -> operator.getBButton() //unfeed note
    );

    ClimberCmd climberCmd = new ClimberCmd(climberSubsystem,
      () -> operator.getLeftY(), //Left climber
      () -> operator.getRightY() //Right climber
    );

    IntakeCmd intakeCmd = new IntakeCmd(intakeSubsystem,
      () -> rightJoystick.getRawButton(1), //intake
      () -> operator.getLeftBumper() //outtake
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    
    driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(leftJoystick.getRawAxis(OIConstants.TranslationY) * (DriverStation.getAlliance().get() == Alliance.Blue ? -1: 1),
         OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(leftJoystick.getRawAxis(OIConstants.TranslationX) * (DriverStation.getAlliance().get() == Alliance.Blue ? -1: 1),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> -rightJoystick.getRawAxis(0)
      );

    driveFieldOrientedAngle = new AbsoluteFieldDrive(
      drivebase,   
      () -> MathUtil.applyDeadband(leftJoystick.getRawAxis(OIConstants.TranslationY) * (DriverStation.getAlliance().get() == Alliance.Blue ? -1: 1),
         OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(leftJoystick.getRawAxis(OIConstants.TranslationX) * (DriverStation.getAlliance().get() == Alliance.Blue ? -1: 1),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> autoAimX()
      );

    driveFieldOrientedAngle.cancel();
    shooterCmd.cancel();
    driveFieldOrientedAnglularVelocity.schedule();

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    shooterSubsystem.setDefaultCommand(shootPosition);
    feederSubsystem.setDefaultCommand(feederWheelCmd);
    climberSubsystem.setDefaultCommand(climberCmd);
    intakeSubsystem.setDefaultCommand(intakeCmd);
  }

  
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(leftJoystick, 3).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(leftJoystick,
                       4).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 2.25:14.75, 5.5), Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0:180)))
                              ));
                              
    new JoystickButton(leftJoystick, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    new JoystickButton(leftJoystick, 2).onTrue(new InstantCommand(() -> setDriveModeAutoAim()));
    new JoystickButton(leftJoystick, 2).onFalse(new InstantCommand(() -> setDriveModeTele()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoName)
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(autoName, true);
  }

  public void setDriveModeAutoAim()
  {
    driveFieldOrientedAnglularVelocity.cancel();
    driveFieldOrientedAngle.schedule();
  }

  public void setDriveModeTele()
  {
    driveFieldOrientedAngle.cancel();
    driveFieldOrientedAnglularVelocity.schedule();
  }

  public void setShootModeAuto(){
    shooterCmd.cancel();
    shootPosition.schedule();
  }

  public void setShootModeTele(){
    shootPosition.cancel();
    shooterCmd.schedule();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public double autoAimX(){
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      return Math.atan((drivebase.getPose().getY() - speakerPose2dBlue.getY())/(drivebase.getPose().getX() - speakerPose2dBlue.getX()));
    }else{
      return Math.atan((drivebase.getPose().getY() - speakerPose2dRed.getY())/(drivebase.getPose().getX() - speakerPose2dRed.getX()))+Math.PI;
    }
  }
/*
  public double autoAimY(){
    if(drivebase.getPose().getTranslation().getDistance(speakerPose2dBlue) < drivebase.getPose().getTranslation().getDistance(speakerPose2dRed)){
      return ;
    }else{
      return ;
    }
  }
  */
  public CameraSubsystem getCameraSubsystem(){
    return cameraSubsystem;
  }

  public ClimberSubsystem getClimberSubsystem(){
    return climberSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem(){
    return intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem(){
    return shooterSubsystem;
  }
}
