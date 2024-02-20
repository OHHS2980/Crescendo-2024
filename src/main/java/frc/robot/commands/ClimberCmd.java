package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCmd extends Command{
    private final ClimberSubsystem climberSubsystem;

    private BooleanSupplier extend;
    private BooleanSupplier retract;


    public ClimberCmd(ClimberSubsystem climberSubsystem,
        BooleanSupplier extend,
        BooleanSupplier retract) {

        this.extend = extend;
        this.retract = retract;

        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        if(extend.getAsBoolean()){ 
            climberSubsystem.setClimberVoltage(1);
        }else if (retract.getAsBoolean()){
            climberSubsystem.setClimberVoltage(-1);
        }else{
            climberSubsystem.setClimberVoltage(0.0);
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