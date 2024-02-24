package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeftMotor = new CANSparkMax(ShooterConstants.flywheelLeftID, MotorType.kBrushless);
    private final CANSparkMax shooterRightMotor = new CANSparkMax(ShooterConstants.flywheelRightID, MotorType.kBrushless);

    private final CANSparkMax shooterArmMotor = new CANSparkMax(ShooterConstants.pivotID, MotorType.kBrushless);

    private PIDController armController = new PIDController(ShooterConstants.armP,ShooterConstants.armI,ShooterConstants.armD);
    private PIDController shooterLeftController = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI,ShooterConstants.shooterD);
    private PIDController shooterRightController = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI,ShooterConstants.shooterD);
    
    private Servo servoRight = new Servo(0);
    private Servo servoLeft = new Servo(1);

    private SparkPIDController shooterControllerSparkLeft;
    private SparkPIDController shooterControllerSparkRight;

    private RelativeEncoder encoderLeft = shooterLeftMotor.getEncoder();
    private RelativeEncoder encoderRight = shooterRightMotor.getEncoder();

    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(ShooterConstants.armEncoderPort);
    private double angle = 240;
    private double velocity;

    private boolean stop = false;

    public ShooterSubsystem(){
        armEncoder.setDistancePerRotation(360);//set units to degrees

        shooterControllerSparkLeft = shooterLeftMotor.getPIDController();
        shooterControllerSparkRight = shooterLeftMotor.getPIDController();

        shooterRightMotor.restoreFactoryDefaults();
        shooterLeftMotor.restoreFactoryDefaults();

        shooterControllerSparkLeft.setSmartMotionMaxVelocity(ShooterConstants.maxSpeed, 0);
        shooterControllerSparkRight.setSmartMotionMaxVelocity(ShooterConstants.maxSpeed, 0);

        armController.setTolerance(1, 0.3);

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

    public void intakePos(){
        setArmPosition(ShooterConstants.intakeArmDeg);
    }
    

    /*       utilities       */

    
    public void setShooterVoltage(double power){
        shooterLeftMotor.set(power);
        shooterRightMotor.set(power);
    }

    public void setServoPos(double posLeft, double posRight){
        servoRight.set(posRight);
        servoLeft.set(posLeft);
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
    
    public double getShooterSpeedLeft(){// radians/sec
        return encoderLeft.getVelocity();
    }
    public double getShooterSpeedRight(){// radians/sec
        return encoderRight.getVelocity();
    }

    public double getPosotion(){// degrees
        return armEncoder.getDistance();
    }

    public double getArmSetpoint(){// degrees
        return angle;
    }

    public double getCalculation(double velocity){
        return shooterLeftController.calculate(encoderLeft.getVelocity(), velocity);
    }

    public void setShooter(double velocity, double angle){
        setShooterSpeed(velocity);
        setArmPosition(angle);
    }

    public void setArmPosition(double angle) {
       this.angle = angle;
       if(angle > 311)angle = 311;
       if(angle < 234)angle = 235;
    }

    public void setShooterSpeed(double velocity) {
        stop = false;
        this.velocity = velocity;
    }

    public void stopShooting(){
        stop = true;
    }

    public void startShooting(){
        setShooterSpeed(-3900);
    }

    public boolean atArmSetpoint(){
        return armController.atSetpoint();
    }

    @Override
    public void periodic(){
        if(stop){
            setShooterVoltage(0);
        }else{
            shooterLeftMotor.set(shooterLeftController.calculate(-encoderLeft.getVelocity(), velocity));
            shooterRightMotor.set(-shooterRightController.calculate(encoderRight.getVelocity(), velocity));   
        }
        setArmVoltage(-armController.calculate(armEncoder.getDistance(), angle));
    }

    
}