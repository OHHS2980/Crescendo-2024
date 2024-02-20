package frc.robot.commands;

import java.time.temporal.ValueRange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCloseCmd extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private double velocity = -4800;
    private double angle = 235;


    public ShootCloseCmd(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        shooterSubsystem.setShooter(velocity, angle);

        
        SmartDashboard.putNumber("Encoder degrees", shooterSubsystem.getPosotion());
        SmartDashboard.putNumber("ShooterSpeedLeft", shooterSubsystem.getShooterSpeedLeft());
        SmartDashboard.putNumber("ShooterSpeedRight", shooterSubsystem.getShooterSpeedRight());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.feedNotes();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.getShooterSpeedLeft() <= velocity && shooterSubsystem.getPosotion() >= angle - 5 && shooterSubsystem.getPosotion() <= angle + 5;
    }
}