package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax leftMotor = new CANSparkMax(IntakeConstants.intakeLeftID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(IntakeConstants.intakeRightID, MotorType.kBrushless);

    
    private DigitalInput initialBeamSensor = new DigitalInput(IntakeConstants.initialBeamSensorPort);
    
    public void setIntakeVoltage(double power){
        leftMotor.set(-power);
        rightMotor.set(-power);
    }

    public double getIntakeSpeed(){
        return leftMotor.getEncoder().getVelocity();
    }

    public void intake(){
        setIntakeVoltage(-1);
    }
    public void stopIntake(){
        setIntakeVoltage(0);
    }

    public boolean getBeamBreak(){
        return initialBeamSensor.get();
    }
}