package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends Command{
    private final ShooterSubsystem shooterSubsystem; 

    private DoubleSupplier shooterSlider;
    private BooleanSupplier shoot;
    private BooleanSupplier feed;
    private BooleanSupplier unFeed;
    private DoubleSupplier armSlider;

    private double armSetpoint;


    public ShooterCmd(ShooterSubsystem shooterSubsystem,
        DoubleSupplier shooterSlider,
        BooleanSupplier shoot,
        BooleanSupplier feed,
        BooleanSupplier unFeed,
        DoubleSupplier armSlider) {

        this.shooterSlider = shooterSlider;
        this.shoot = shoot;
        this.feed = feed;
        this.unFeed = unFeed;
        this.armSlider = armSlider;

        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        armSetpoint = shooterSubsystem.getPosotion();
    }
    
    @Override
    public void execute() {
        if(shoot.getAsBoolean()){
            shooterSubsystem.setShooterSpeed(shooterSlider.getAsDouble() * ShooterConstants.maxSpeed);
            //shooterSubsystem.setShooterVoltage(shooterSlider.getAsDouble());
        }else{
            shooterSubsystem.setShooterVoltage(0);
            //shooterSubsystem.setShooterVoltage(0);
        }

        //set arm setpoint to the slider clamped to 0.4-0.5   (armSlider.getAsDouble()+1)*0.05)+0.4)
        
        
        //increment setpoint a very small amount based on the slider
        
        
        shooterSubsystem.setArmPosition(((armSlider.getAsDouble() + 1)/2)*(65) + 188);
        

        if(feed.getAsBoolean()){ 
            shooterSubsystem.feedNotes();
        }else if(unFeed.getAsBoolean()){
            shooterSubsystem.unfeedNotes();
        }else{
            shooterSubsystem.stopFeed();
        }
        SmartDashboard.putNumber("Encoder degrees", shooterSubsystem.getPosotion());
        SmartDashboard.putNumber("ShooterSpeedLeft", shooterSubsystem.getShooterSpeedLeft());
        SmartDashboard.putNumber("ShooterSpeedRight", shooterSubsystem.getShooterSpeedRight());
        SmartDashboard.putNumber("ShooterTaget", shooterSlider.getAsDouble() * ShooterConstants.maxSpeed);
    
        SmartDashboard.putNumber("ArmSetpoint", armSetpoint);

    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}