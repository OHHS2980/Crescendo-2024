package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class feederWheelCmd extends Command{
    private final FeederSubsystem feederSubsystem;

    private BooleanSupplier shoot;
    private BooleanSupplier intake;
    private BooleanSupplier center;

    public feederWheelCmd(FeederSubsystem feederSubsystem, BooleanSupplier shoot, BooleanSupplier intake, BooleanSupplier center) {

        this.feederSubsystem = feederSubsystem;

        this.shoot = shoot;
        this.intake = intake;
        this.center = center;
        
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        
        if(shoot.getAsBoolean() || intake.getAsBoolean() || center.getAsBoolean()){
            if(shoot.getAsBoolean()){
                feederSubsystem.feedNotes();
            }

            if(intake.getAsBoolean()){
                if(feederSubsystem.getBeamBreak()){
                    feederSubsystem.feedNotes();
                }else{
                    feederSubsystem.stopFeed();
                }

            }

            if(center.getAsBoolean()){
                
            }

        }else{
            feederSubsystem.stopFeed();
        }
        
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}