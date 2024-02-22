package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax leftMotor = new CANSparkMax(ClimberConstants.climberLeftID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ClimberConstants.climberRightID, MotorType.kBrushless);

    public ClimberSubsystem(){
        //set units to inches
        leftMotor.getEncoder().setPositionConversionFactor(ClimberConstants.positionConversionFactor);
        rightMotor.getEncoder().setPositionConversionFactor(ClimberConstants.positionConversionFactor);

        setClimberValues(ClimberConstants.climberP, ClimberConstants.climberI, ClimberConstants.climberD);
    }


    public void home(){
        setClimberPos(ClimberConstants.stowedEncoderVal, ClimberConstants.stowedEncoderVal);
    }

    public void maxHeight(){
        setClimberPos(ClimberConstants.maxEncoderVal, ClimberConstants.maxEncoderVal);
    }



    /*       utilities       */



    public void setClimberValues(double p, double i, double d){
        leftMotor.getPIDController().setP(p);
        leftMotor.getPIDController().setP(i);
        leftMotor.getPIDController().setP(d);
        
        rightMotor.getPIDController().setP(p);
        rightMotor.getPIDController().setP(i);
        rightMotor.getPIDController().setP(d);
    }
    
    public void setClimberVoltage(double power){
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void setLeftClimberVoltage(double power){
        leftMotor.set(power);
    }

    public void setRightClimberVoltage(double power){
        rightMotor.set(power);
    }

    public void setClimberPos(double leftPos, double rightPos){//inches
        leftMotor.getPIDController().setReference(leftPos, CANSparkMax.ControlType.kSmartMotion);
        rightMotor.getPIDController().setReference(rightPos, CANSparkMax.ControlType.kSmartMotion);
    }

    public double getLeftClimberPos(){//inches
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightClimberPos(){//inches
        return rightMotor.getEncoder().getPosition();
    }
}