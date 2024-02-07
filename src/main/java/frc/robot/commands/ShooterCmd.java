package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends Command{
    private final ShooterSubsystem shooterSubsystem; 

    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(shooterConstants.armEncoderPort);

    private PIDController armController;

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

        armEncoder.setDistancePerRotation(360);//set units to degrees
        armSetpoint = armEncoder.getDistance();

        armController = new PIDController(shooterConstants.shooterP,shooterConstants.shooterI,shooterConstants.shooterD);
        shooterSubsystem.setShooterValues(shooterConstants.armP, shooterConstants.armI, shooterConstants.armD);;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        if(shoot.getAsBoolean()){
            shooterSubsystem.setShooterSpeed(shooterSlider.getAsDouble() * shooterConstants.maxSpeed);
        }else{
            shooterSubsystem.setShooterVoltage(0);
        }

        //set arm setpoint to the slider clamped to 0.4-0.5   (armSlider.getAsDouble()+1)*0.05)+0.4)
        
        
        //increment setpoint a very small amount based on the slider
        armSetpoint += MathUtil.applyDeadband(armSlider.getAsDouble(), 0.25) * 0.005;
        shooterSubsystem.setArmVoltage(Math.min(armController.calculate(armEncoder.getDistance(), armSetpoint) ,  0.25));
        

        if(feed.getAsBoolean()){ 
            shooterSubsystem.feedNotes();
        }else{
            shooterSubsystem.stopFeed();
        }
        SmartDashboard.putNumber("Encoder degrees", armEncoder.getDistance());
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