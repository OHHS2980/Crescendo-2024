package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeftMotor = new CANSparkMax(ShooterConstants.flywheelLeftID, MotorType.kBrushless);
    private final CANSparkMax shooterRightMotor = new CANSparkMax(ShooterConstants.flywheelRightID, MotorType.kBrushless);

    private final CANSparkMax feedRightMotor = new CANSparkMax(ShooterConstants.stagingRightID, MotorType.kBrushed);
    private final CANSparkMax feedLeftMotor = new CANSparkMax(ShooterConstants.stagingLeftID, MotorType.kBrushed);

    private final CANSparkMax shooterArmMotor = new CANSparkMax(ShooterConstants.pivotID, MotorType.kBrushless);

    private PIDController armController = new PIDController(ShooterConstants.armP,ShooterConstants.armI,ShooterConstants.armD);
    private PIDController shooterLeftController = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI,ShooterConstants.shooterD);
    private PIDController shooterRightController = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI,ShooterConstants.shooterD);

    private SparkPIDController shooterControllerSparkLeft;
    private SparkPIDController shooterControllerSparkRight;

    private RelativeEncoder encoderLeft = shooterLeftMotor.getEncoder();
    private RelativeEncoder encoderRight = shooterRightMotor.getEncoder();

    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(ShooterConstants.armEncoderPort);

    public ShooterSubsystem(){
        armEncoder.setDistancePerRotation(360);//set units to degrees

        shooterControllerSparkLeft = shooterLeftMotor.getPIDController();
        shooterControllerSparkRight = shooterLeftMotor.getPIDController();

        shooterRightMotor.restoreFactoryDefaults();
        shooterLeftMotor.restoreFactoryDefaults();

        shooterControllerSparkLeft.setSmartMotionMaxVelocity(ShooterConstants.maxSpeed, 0);
        shooterControllerSparkRight.setSmartMotionMaxVelocity(ShooterConstants.maxSpeed, 0);

        setShooterValues(ShooterConstants.shooterP,ShooterConstants.shooterI,ShooterConstants.shooterD);
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

    public void setShooterSpeed(double velocity){// radians/sec
        shooterLeftMotor.set(shooterLeftController.calculate(-encoderLeft.getVelocity(), velocity));
        shooterRightMotor.set(-shooterRightController.calculate(encoderRight.getVelocity(), velocity));
        //shooterControllerSparkLeft.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        //shooterControllerSparkRight.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setShooterValues(double p, double i, double d){
        shooterControllerSparkLeft.setP(p);
        shooterControllerSparkLeft.setP(i);
        shooterControllerSparkLeft.setP(d);
        
        shooterControllerSparkRight.setP(p);
        shooterControllerSparkRight.setP(i);
        shooterControllerSparkRight.setP(d);
    }

    public void setArmVoltage(double power){
        shooterArmMotor.set(Math.min(power ,  0.25));
    }

    public void setArmPosition(double position){// degrees
        setArmVoltage(-armController.calculate(armEncoder.getDistance(), position));
    }

    public void feedNotes(){
        feedLeftMotor.set(-1);
        feedRightMotor.set(1);
    }

    public void unfeedNotes(){
        feedLeftMotor.set(1);
        feedRightMotor.set(-1);
    }

    public void stopFeed(){
        feedLeftMotor.set(0);
        feedRightMotor.set(0);
    }
    
    public double getShooterSpeedLeft(){// radians/sec
        return encoderLeft.getVelocity();
    }
    public double getShooterSpeedRight(){// radians/sec
        return encoderRight.getVelocity();
    }

    public double getPosotion(){// degrees
        return armEncoder.getDistance();
    }

    public double getCalculation(double velocity){
        return shooterLeftController.calculate(encoderLeft.getVelocity(), velocity);
    }
}