// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;

    public static final double TURN_CONSTANT    = 6;
  }
  public static class OIConstants
  {
    public static final int TranslationY  = 1;
    public static final int TranslationX  = 0;
    public static final int Rotation = 0;

    public static final int DriveJoystick  = 0;
    public static final int TurnJoystick = 1;
    public static final int operatorJoystick = 2;
  }

  public static class ShooterConstants
  {
    public static final int flywheelLeftID = 14;
    public static final int flywheelRightID = 15;
    public static final int stagingLeftID = 12;
    public static final int stagingRightID = 13;
    public static final int pivotID = 11;

    public static final double maxSpeed = 4800;

    public static final double StowedArmDeg = 0;//tune these
    public static final double subwooferArmDeg = 0;
    public static final double ampArmDeg = 0;

    public static final double ampShooterVel = 0;
    public static final double subwooferShooterVel = 0;

    public static final int armEncoderPort = 0;

    public static double shooterP =  0.00018;
    public static double shooterI = 0;
    public static double shooterD = 0;

    public static double armP = 0.005;
    public static double armI = 0.0;
    public static double armD = 0.0;
  }

  public static class IntakeConstants
  {
    public static final int intakeLeftID = 9;
    public static final int intakeRightID = 10;

    public static final int initialBeamSensorPort = 1;
    public static final int finalBeamSensorPort = 2;
  }

  public static class ClimberConstants
  {
    public static final int climberLeftID = 16;
    public static final int climberRightID = 17;

    public static final double stowedEncoderVal = 0;
    public static final double maxEncoderVal = 20;

    public static final double positionConversionFactor = 2.47; //diameter of spool to convert rotations to inches 

    public static double climberP = 0.005;
    public static double climberI = 0.0;
    public static double climberD = 0.0;
  }

}
