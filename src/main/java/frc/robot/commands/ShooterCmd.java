package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends Command{
    private final ShooterSubsystem shooterSubsystem; 

    private DoubleSupplier shooterSlider;
    private DoubleSupplier armSlider;
    private BooleanSupplier shoot;
    private BooleanSupplier feed;

    private double armSetpoint;


    public ShooterCmd(ShooterSubsystem shooterSubsystem,
        DoubleSupplier shooterSlider,
        BooleanSupplier shoot,
        BooleanSupplier feed,
        DoubleSupplier armSlider) {

        this.shooterSlider = shooterSlider;
        this.armSlider = armSlider;
        this.shoot = shoot;
        this.feed = feed;

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
        }else{
            shooterSubsystem.setShooterVoltage(0);
        }

        //set arm setpoint to the slider clamped to 0.4-0.5   (armSlider.getAsDouble()+1)*0.05)+0.4)
        
        
        //increment setpoint a very small amount based on the slider
        armSetpoint += MathUtil.applyDeadband(armSlider.getAsDouble(), 0.25) * 0.005;
        shooterSubsystem.setArmPosition(armSetpoint);
        

        if(feed.getAsBoolean()){ 
            shooterSubsystem.feedNotes();
        }else{
            shooterSubsystem.stopFeed();
        }
        SmartDashboard.putNumber("Encoder degrees", shooterSubsystem.getPosotion());
        SmartDashboard.putNumber("ShooterSpeed", shooterSubsystem.getShooterSpeed());
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}