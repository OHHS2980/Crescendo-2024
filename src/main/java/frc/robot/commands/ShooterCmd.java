package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends Command{
    private final ShooterSubsystem shooterSubsystem; 

    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);

    private PIDController armController = new PIDController(0.005,0,0);
    
    private PIDController shooterController = new PIDController(0.005,0,0);

    private DoubleSupplier shooterSlider;
    private BooleanSupplier shoot;
    private BooleanSupplier feed;

    public ShooterCmd(ShooterSubsystem shooterSubsystem,
        DoubleSupplier shooterSlider,
        BooleanSupplier shoot,
        BooleanSupplier feed) {

        this.shooterSlider = shooterSlider;
        this.shoot = shoot;
        this.feed = feed;

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        if(shoot.getAsBoolean()){
            shooterSubsystem.setShooterVoltage(shooterController.calculate(shooterSubsystem.getShooterSpeed(), shooterSlider.getAsDouble() * shooterConstants.maxSpeed));
        }else{
            shooterSubsystem.setShooterVoltage(0);
        }
        shooterSubsystem.setArmVoltage(armController.calculate(armEncoder.get(), 0));

        if(feed.getAsBoolean()){
            shooterSubsystem.feedNotes();
        }else{
            shooterSubsystem.stopFeed();
        }
        SmartDashboard.putNumber("Encoder", armEncoder.get());
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