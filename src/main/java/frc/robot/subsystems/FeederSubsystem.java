package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class FeederSubsystem extends SubsystemBase{
    private final CANSparkMax feedRightMotor = new CANSparkMax(ShooterConstants.stagingRightID, MotorType.kBrushless);
    private final CANSparkMax feedLeftMotor = new CANSparkMax(ShooterConstants.stagingLeftID, MotorType.kBrushless);

    
    private DigitalInput initialBeamSensor = new DigitalInput(IntakeConstants.initialBeamSensorPort);

    public FeederSubsystem(){
    }

    public void feedNotes(){
        feedLeftMotor.set(-0.65);
        feedRightMotor.set(0.65);
    }

    public void unfeedNotes(){
        feedLeftMotor.set(0.65);
        feedRightMotor.set(-0.65);
    }

    public void stopFeed(){
        feedLeftMotor.set(0);
        feedRightMotor.set(0);
    }

    public boolean getBeamBreak(){
        return initialBeamSensor.get();
    }
}