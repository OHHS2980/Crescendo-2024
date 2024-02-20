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
    private BooleanSupplier ampArm;

    private double armSetpoint;
    private boolean ampIn = true;


    public ShooterCmd(ShooterSubsystem shooterSubsystem,
        DoubleSupplier shooterSlider,
        BooleanSupplier shoot,
        BooleanSupplier feed,
        BooleanSupplier unFeed,
        DoubleSupplier armSlider,
        BooleanSupplier ampArm) {

        this.shooterSlider = shooterSlider;
        this.shoot = shoot;
        this.feed = feed;
        this.unFeed = unFeed;
        this.armSlider = armSlider;
        this.ampArm = ampArm;

        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        armSetpoint = shooterSubsystem.getArmSetpoint();
    }
    
    @Override
    public void execute() {
        if(shoot.getAsBoolean()){
            shooterSubsystem.setShooterSpeed(shooterSlider.getAsDouble() * ShooterConstants.maxSpeed);
            //shooterSubsystem.setShooterVoltage(shooterSlider.getAsDouble());
        }else{
            shooterSubsystem.setShooterSpeed(0);
            //shooterSubsystem.setShooterVoltage(0);
        }

        shooterSubsystem.setArmPosition(((armSlider.getAsDouble() + 1)/2)*(311-234) + 234);

        armSetpoint = shooterSubsystem.getArmSetpoint();

        if(ampArm.getAsBoolean()){
            ampIn = !ampIn;
        }

        if(ampIn){
            shooterSubsystem.setServoPos(0.16, 0.79);
        }else{
            shooterSubsystem.setServoPos(0.915,0.04);
        }

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