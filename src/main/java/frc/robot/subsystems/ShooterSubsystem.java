package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeftMotor = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax shooterRightMotor = new CANSparkMax(15, MotorType.kBrushless);

    private final CANSparkMax feedRightMotor = new CANSparkMax(12, MotorType.kBrushed);
    private final CANSparkMax feedLeftMotor = new CANSparkMax(13, MotorType.kBrushed);

    
    private final CANSparkMax shooterArmMotor = new CANSparkMax(11, MotorType.kBrushless);
    
    public void setShooterVoltage(double power){
        shooterLeftMotor.set(power);
        shooterRightMotor.set(power);
    }

    public void setArmVoltage(double power){
        shooterArmMotor.set(power);
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