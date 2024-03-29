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
    private DoubleSupplier armSlider;
    private BooleanSupplier ampArm;

    private double armSetpoint;
    private boolean ampIn = true;


    public ShooterCmd(ShooterSubsystem shooterSubsystem,
        DoubleSupplier shooterSlider,
        BooleanSupplier shoot,
        DoubleSupplier armSlider,
        BooleanSupplier ampArm) {

        this.shooterSlider = shooterSlider;
        this.shoot = shoot;
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



        shooterSubsystem.setArmVoltage(armSlider.getAsDouble());

        armSetpoint = shooterSubsystem.getArmSetpoint();

        if(ampArm.getAsBoolean()){
            ampIn = !ampIn;
        }

        if(ampIn){
            shooterSubsystem.setServoPos(0.16, 0.79);
        }else{
            shooterSubsystem.setServoPos(0.915,0.04);
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