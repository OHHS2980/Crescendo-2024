package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    
    private DigitalInput initialBeamSensor = new DigitalInput(IntakeConstants.initialBeamSensorPort);
    private DigitalInput finalBeamSensor = new DigitalInput(IntakeConstants.finalBeamSensorPort);

    private BooleanSupplier intake;
    private BooleanSupplier outtake;


    public IntakeCmd(IntakeSubsystem intakeSubsystem,
        BooleanSupplier intake,
        BooleanSupplier outtake) {

        this.intake = intake;
        this.outtake = outtake;

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        if(intake.getAsBoolean()){ 
            intakeSubsystem.setIntakeVoltage(1);
            SmartDashboard.putBoolean("INtaking",true);
        }else if (outtake.getAsBoolean()){
            intakeSubsystem.setIntakeVoltage(-1);
        }else{
            intakeSubsystem.setIntakeVoltage(0);
            SmartDashboard.putBoolean("INtaking",false);

        }

        SmartDashboard.putNumber("IntakeSpeed", intakeSubsystem.getIntakeSpeed());

        SmartDashboard.putBoolean("InitialBeamBreak", initialBeamSensor.get());
        SmartDashboard.putBoolean("FinalBeamBreak", finalBeamSensor.get());
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}