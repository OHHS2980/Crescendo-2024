package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class autoIntakeCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;

    public autoIntakeCmd(IntakeSubsystem intakeSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intake();
    }
    
    @Override
    public void execute() {
        
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getBeamBreak();
    }
}