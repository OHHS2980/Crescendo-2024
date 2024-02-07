package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeftMotor = new CANSparkMax(ShooterConstants.flywheelLeftID, MotorType.kBrushless);
    private final CANSparkMax shooterRightMotor = new CANSparkMax(ShooterConstants.flywheelRightID, MotorType.kBrushless);

    private final CANSparkMax feedRightMotor = new CANSparkMax(ShooterConstants.stagingRightID, MotorType.kBrushless);
    private final CANSparkMax feedLeftMotor = new CANSparkMax(ShooterConstants.stagingLeftID, MotorType.kBrushless);

    private final CANSparkMax shooterArmMotor = new CANSparkMax(ShooterConstants.pivotID, MotorType.kBrushless);

    private PIDController armController = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI,ShooterConstants.shooterD);
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(ShooterConstants.armEncoderPort);

    public ShooterSubsystem(){
        armEncoder.setDistancePerRotation(360);//set units to degrees

        setShooterValues(ShooterConstants.armP, ShooterConstants.armI, ShooterConstants.armD);
    }
    
    public void home(){
        setArmPosition(ShooterConstants.StowedArmDeg);
        setShooterSpeed(0);
    }

    public void amp(){
        setArmPosition(ShooterConstants.ampArmDeg);
        setShooterSpeed(ShooterConstants.ampShooterVel);
    }

    public void subwoofer(){
        setArmPosition(ShooterConstants.subwooferArmDeg);
        setShooterSpeed(ShooterConstants.subwooferShooterVel);
    }
    

    /*       utilities       */

    
    public void setShooterVoltage(double power){
        shooterLeftMotor.set(power);
        shooterRightMotor.set(power);
    }

    public void setShooterSpeed(double velocity){
        shooterLeftMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kSmartVelocity);
        shooterRightMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kSmartVelocity);
    }

    public void setArmVoltage(double power){
        shooterArmMotor.set(Math.min(power ,  0.25));
    }

    public void setArmPosition(double position){
        setArmVoltage(armController.calculate(armEncoder.getDistance(), position));
    }

    public void setShooterValues(double p, double i, double d){
        shooterLeftMotor.getPIDController().setP(p);
        shooterLeftMotor.getPIDController().setP(i);
        shooterLeftMotor.getPIDController().setP(d);
        
        shooterRightMotor.getPIDController().setP(p);
        shooterRightMotor.getPIDController().setP(i);
        shooterRightMotor.getPIDController().setP(d);
    }

    public void feedNotes(){
        feedLeftMotor.set(1);
        feedRightMotor.set(1);
    }
    public void stopFeed(){
        feedLeftMotor.set(0);
        feedRightMotor.set(0);
    }
    
    public double getShooterSpeed(){
        return shooterLeftMotor.getEncoder().getVelocity();
    }

    public double getPosotion(){//degrees
        return armEncoder.getDistance();
    }
}