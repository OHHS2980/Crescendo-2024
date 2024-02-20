package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class feederWheelCmd extends Command{
    private final ShooterSubsystem shooterSubsystem;

    private double armSetpoint;
    private Timer Timer;
    private double time;
    private boolean originalBeamBreak;


    public feederWheelCmd(ShooterSubsystem shooterSubsystem) {

        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {/*
        if(shooterSubsystem.getBeamBreak()){
            originalBeamBreak = true;
        }else{
            shooterSubsystem.feedNotes();
            originalBeamBreak = false;
        }*/
    }
    
    @Override
    public void execute() {/*
        if(originalBeamBreak){
            
        }else{
            if(shooterSubsystem.getBeamBreak()){
                shooterSubsystem.stopFeed();
            }
        }*/
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}