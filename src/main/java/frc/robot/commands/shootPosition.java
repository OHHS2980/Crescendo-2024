package frc.robot.commands;

import java.time.temporal.ValueRange;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class shootPosition extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private Supplier<Pose2d> robotPos;
    private final Translation2d speakerPose2dBlue = new Translation2d(0.0, 5.5);
    private final Translation2d speakerPose2dRed = new Translation2d(16.54, 5.5);

    private double velocityShoot = -3900;
    private double velocityAmp = -4800;
    private double angle  = 290;

    private Boolean autoAim = false;
    private Boolean ampOut = false;

    private BooleanSupplier shoot;
    private BooleanSupplier amp;
    private BooleanSupplier intake;
    private BooleanSupplier aim;
    private BooleanSupplier source;


    public shootPosition(Supplier<Pose2d> robotPos, ShooterSubsystem shooterSubsystem, BooleanSupplier shoot, BooleanSupplier amp, BooleanSupplier intake, BooleanSupplier aim, BooleanSupplier source) {
        this.robotPos = robotPos;

        this.shoot = shoot;
        this.amp = amp;
        this.intake = intake;
        this.aim = aim;
        this.source = source;

        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if(aim.getAsBoolean()){
            autoAim = true;
            ampOut = false;
        }

        if(amp.getAsBoolean()){
            ampOut = true;
            autoAim = false;
        }

        if(autoAim){
            angle = -180*Math.atan(1.85/Math.min(robotPos.get().getTranslation().getDistance(speakerPose2dBlue), robotPos.get().getTranslation().getDistance(speakerPose2dRed)))/Math.PI + 339;
            angle = Math.min(Math.max(angle, 257), 310);
        }

        if(ampOut){
            shooterSubsystem.setServoPos(0.915,0.04);
            angle = 235;
        }else{
            shooterSubsystem.setServoPos(0.16, 0.79);
        }

        if(intake.getAsBoolean()){
            angle = 276;
            autoAim = false;
            ampOut = false;
        }

        if(shoot.getAsBoolean()){
            if(ampOut){
                shooterSubsystem.setShooterSpeed(velocityAmp);
            }else{
                shooterSubsystem.setShooterSpeed(velocityShoot);
            } 
        }else{
            shooterSubsystem.stopShooting();
        }
        
        shooterSubsystem.setArmPosition(angle);

        SmartDashboard.putNumber("ArmSetpoint", angle);
        SmartDashboard.putNumber("Encoder degrees", shooterSubsystem.getPosotion());
        SmartDashboard.putNumber("ShooterSpeedLeft", shooterSubsystem.getShooterSpeedLeft());
        SmartDashboard.putNumber("ShooterSpeedRight", shooterSubsystem.getShooterSpeedRight());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}