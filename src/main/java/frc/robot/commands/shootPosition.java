package frc.robot.commands;

import java.time.temporal.ValueRange;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class shootPosition extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private Supplier<Pose2d> robotPos;
    private final Translation2d speakerPose2dBlue = new Translation2d(0.0, 5.5);
    private final Translation2d speakerPose2dRed = new Translation2d(16.54, 5.5);
    private double velocity = -3900;
    private double angle;


    public shootPosition(Supplier<Pose2d> robotPos, ShooterSubsystem shooterSubsystem) {
        this.robotPos = robotPos;

        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        angle = -180*Math.atan(1.75/Math.min(robotPos.get().getTranslation().getDistance(speakerPose2dBlue), robotPos.get().getTranslation().getDistance(speakerPose2dRed)))/Math.PI + 285.2;
        angle = Math.min(Math.max(angle, 188), 261.7);

        shooterSubsystem.setArmPosition(angle);

        SmartDashboard.putNumber("ArmSetpoint", 180*Math.atan(1.75/Math.min(robotPos.get().getTranslation().getDistance(speakerPose2dBlue), robotPos.get().getTranslation().getDistance(speakerPose2dRed)))/Math.PI);
        SmartDashboard.putNumber("Encoder degrees", shooterSubsystem.getPosotion());
        SmartDashboard.putNumber("ShooterSpeedLeft", shooterSubsystem.getShooterSpeedLeft());
        SmartDashboard.putNumber("ShooterSpeedRight", shooterSubsystem.getShooterSpeedRight());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}