package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCmd extends Command{
    private final ClimberSubsystem climberSubsystem;

    private DoubleSupplier rightClimber;
    private DoubleSupplier leftClimber;


    public ClimberCmd(ClimberSubsystem climberSubsystem,
        DoubleSupplier leftClimber,
        DoubleSupplier rightClimber) {

        this.rightClimber = rightClimber;
        this.leftClimber = leftClimber;

        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        if(rightClimber.getAsDouble() > 0.1){
            climberSubsystem.setRightClimberVoltage(0.5);
        }else if(rightClimber.getAsDouble() < -0.1){
            climberSubsystem.setRightClimberVoltage(-0.5);
        }else{
            climberSubsystem.setRightClimberVoltage(0);
        }

        if(leftClimber.getAsDouble() > 0.1){
            climberSubsystem.setLeftClimberVoltage(0.5);
        }else if(leftClimber.getAsDouble() < -0.1){
            climberSubsystem.setLeftClimberVoltage(-0.5);
        }else{
            climberSubsystem.setLeftClimberVoltage(0);
        }

        SmartDashboard.putNumber("LeftClimberPos", climberSubsystem.getLeftClimberPos());
        SmartDashboard.putNumber("RightClimberPos", climberSubsystem.getRightClimberPos());
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}