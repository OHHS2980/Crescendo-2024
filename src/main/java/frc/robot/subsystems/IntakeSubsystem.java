package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
        
    public IntakeSubsystem(){
        leftMotor = new CANSparkMax(IntakeConstants.intakeLeftID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(IntakeConstants.intakeRightID, MotorType.kBrushless);
    }

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

}