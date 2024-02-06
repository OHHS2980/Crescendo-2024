package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax leftMotor = new CANSparkMax(intakeConstants.intakeLeftID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(intakeConstants.intakeRightID, MotorType.kBrushless);
    
    public void setIntakeVoltage(double power){
        leftMotor.set(power);
        rightMotor.set(-power);
    }

    public double getIntakeSpeed(){
        return leftMotor.getEncoder().getVelocity();
    }
}