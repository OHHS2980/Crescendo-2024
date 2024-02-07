package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeftMotor = new CANSparkMax(shooterConstants.flywheelLeftID, MotorType.kBrushless);
    private final CANSparkMax shooterRightMotor = new CANSparkMax(shooterConstants.flywheelRightID, MotorType.kBrushless);

    private final CANSparkMax feedRightMotor = new CANSparkMax(shooterConstants.stagingRightID, MotorType.kBrushless);
    private final CANSparkMax feedLeftMotor = new CANSparkMax(shooterConstants.stagingLeftID, MotorType.kBrushless);

    
    private final CANSparkMax shooterArmMotor = new CANSparkMax(shooterConstants.pivotID, MotorType.kBrushless);
    
    public void setShooterVoltage(double power){
        shooterLeftMotor.set(power);
        shooterRightMotor.set(power);
    }

    public void setShooterSpeed(double velocity){
        shooterLeftMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kSmartVelocity);
        shooterRightMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kSmartVelocity);
    }

    public void setArmVoltage(double power){
        shooterArmMotor.set(power);
    }

    public void setShooterValues(double p, double i, double d){
        shooterLeftMotor.getPIDController().setP(p);
        shooterLeftMotor.getPIDController().setP(i);
        shooterLeftMotor.getPIDController().setP(d);
        
        shooterRightMotor.getPIDController().setP(p);
        shooterRightMotor.getPIDController().setP(i);
        shooterRightMotor.getPIDController().setP(d);
    }

    public double getShooterSpeed(){
        return shooterLeftMotor.getEncoder().getVelocity();
    }

    public void feedNotes(){
        feedLeftMotor.set(1);
        feedRightMotor.set(1);
    }
    public void stopFeed(){
        feedLeftMotor.set(0);
        feedRightMotor.set(0);
    }
}