// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.library.SwerveData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    //public static final String kCanBus = "CANivore";
    public static final String kCanBus = "rio";

    public static final int kFrontLeftDriveMotorPort = 42;
    public static final int kRearLeftDriveMotorPort = 40;
    public static final int kFrontRightDriveMotorPort = 43;
    public static final int kRearRightDriveMotorPort = 41;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kRearRightDriveMotorReversed = true;

    public static final int kFrontLeftTurningMotorPort = 32;
    public static final int kRearLeftTurningMotorPort = 30;
    public static final int kFrontRightTurningMotorPort = 33;
    public static final int kRearRightTurningMotorPort = 31;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kRearLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kRearRightTurningMotorReversed = true;

    public static final int kFrontLeftTurningEncoderPorts = 53;
    public static final int kRearLeftTurningEncoderPorts = 51;
    public static final int kFrontRightTurningEncoderPorts = 52;
    public static final int kRearRightTurningEncoderPorts = 50;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.65405;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.43815;
    
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final NeutralModeValue driveMode = NeutralModeValue.Coast;
    public static final NeutralModeValue turnMode = NeutralModeValue.Brake;
    public static final double maxVoltage = 12.0;
    public static final double maxSpeed = 2.0; //4.36 m/s
    public static final double maxAngularSpeed = Math.PI*4; // 1/2 Rotation/Sec
    public static final double rotKp = 0.01;
    public static final double rotKi = 0.02;
    public static final double rotKd = 0.0;
    public static final double rotToleranceDeg = 1;
    public static final double rotToleranceVel = 10; // Deg/sec
    public static final double rotMaxOutput = 0.5;
    public static final double stickDeadband = 0.1;

    public static final double speedScaleMin = 0.10;
    public static final double speedScaleMax = 0.80;

    public static final double rotationScaleMin = 0.025;
    public static final double rotationScaleMax = 0.200;

    public static final double MetersPerInch = 1/39.37008;
    public static final int kCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 0;

    public static final boolean kGyroReversed = true;

    public static final boolean kuseAbsEnc = true;
// if we have to replace a swerve module we nneed to change the number to 0 
// and then align the wheel and see the values from the driver station and put those numbers in here
// there are going to be negative numbers
    public static final SwerveData SDFrontLeft = new SwerveData("FL", 
    kFrontLeftDriveMotorPort, 
    kFrontLeftDriveMotorReversed, 
    kFrontLeftTurningMotorPort, 
    kFrontLeftTurningMotorReversed, 
    kFrontLeftTurningEncoderPorts,
    kFrontLeftTurningEncoderReversed,
    -0.923,
    kuseAbsEnc);

    public static final SwerveData SDFrontRight = new SwerveData("FR", 
    kFrontRightDriveMotorPort, 
    kFrontRightDriveMotorReversed, 
    kFrontRightTurningMotorPort, 
    kFrontRightTurningMotorReversed, 
    kFrontRightTurningEncoderPorts,
    kFrontRightTurningEncoderReversed,
    -0.240,
    kuseAbsEnc);

  public static final SwerveData SDRearLeft = new SwerveData("RL", 
    kRearLeftDriveMotorPort, 
    kRearLeftDriveMotorReversed, 
    kRearLeftTurningMotorPort, 
    kRearLeftTurningMotorReversed, 
    kRearLeftTurningEncoderPorts,
    kRearLeftTurningEncoderReversed,
    -0.595, //-.653
    kuseAbsEnc);

  public static final SwerveData SDRearRight = new SwerveData("RR", 
    kRearRightDriveMotorPort, 
    kRearRightDriveMotorReversed, 
    kRearRightTurningMotorPort, 
    kRearRightTurningMotorReversed, 
    kRearRightTurningEncoderPorts, 
    kRearRightTurningEncoderReversed,
    -0.106,
    kuseAbsEnc);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechControllerPort = 1;

    public static final int kUpDPad = 180;
    public static final int kDownDPad = 0;
    public static final int kLeftDPad = 270;
    public static final int kRightDPad = 90;
  }

  public static final class SwerveConstants {

    public static final double steerKp = 0.55;
    public static final double steerKi = 0.15;
    public static final double steerKd = 0;

    public static final double steerMax_RadPS = Math.PI;
    public static final double steerMax_RadPSSq = Math.pow(steerMax_RadPS,2);
    public static final double steerRatio = 396.0/35.0; // Per Westcoast products website
    public static final double steer_RevPRad = steerRatio/(2.0*Math.PI); 
    
    
    public static final double driveDistanceCntsPMeter = 7.73/(0.1016*Math.PI); //49907
    public static final double driveRawVelocityToMPS = 4990.68;
  }


  public static class ClimbConstants {

    public static final String kCanBus = "rio";
    public static final int kMotorPort = 26;

    public static final boolean kClimbMotorInverted = true;
    public static final NeutralModeValue kIdleMode = NeutralModeValue.Coast; 
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;

    public static final double kPPos = 0.01;
    public static final double kIPos = 0.00;
    public static final double kDPos = 0.0;
    public static final double kPosErrTolerance = 1;

    public static final double climbMax_RadPS = Math.PI/0.5;
    public static final double climbMax_RadPSSq = Math.pow(climbMax_RadPS,2);

    public static final double k0Lock = 0; // Up
    public static final double k1Lock = -990; // Right (hook)
    public static final double k2Lock = 1570; // Left (climb)

  }


  public static class ElevatorConstants {
    public static final int kLeftMotorPort = 21;
    public static final int kRigthMotorPort = 20;

    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 10;
    public static final IdleMode kIdleMode = IdleMode.kBrake;

    public static final double kPPos = 0.2;
    public static final double kIPos= 0.01;
    public static final double kDPos = 0;
    public static final double kPosErrTolerance = 1;

    public static final double kMaxVel = Math.PI/0.3; //RadPerSec
    public static final double kMaxAcc = Math.pow(kMaxVel,4);; //RadPerSecSqrd
    public static final double kGearRatio = 4.45 * Math.PI;
    
    public static final boolean kLeftMoterInverted = true;
    public static final boolean kRightMotorInverted = false;

    // Max 262
    public static final double k0Lock = 0.0;     // Down
    public static final double k1Lock = 110.0;   // 1st level
    public static final double k2Lock = 0.0;     // Intake
    public static final double k3Lock = 240.0;   // 2nd level
    public static final double k4Lock = 255.0;   // 3rd level
  }


  public static class CoralConstants {
    public static final int kMotorPort = 23;
    public static final int kMotorHingePort = 22;
    public static final boolean kCoralMotorInverted = true;
    public static final boolean kCoralHingeMotorInverted = true;

    public static final IdleMode kCoralIdleMode = IdleMode.kBrake;
    public static final IdleMode kCoralHingeIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kCoralCurrentLimit = 30;
    public static final int kCoralHingeCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 0;
    public static final double kMaxOutputOut = 1.0;
    public static final double kMaxOutputIn = 0.5;
        
    public static final double kGearBoxRatio = 1/7.0;
    public static final double kEncoderRpmToWheelRpm = kGearBoxRatio;

    public static final double kMaxVel = 1;

    public static final double kPPos = 0.0005;
    public static final double kIPos = 0.0; //.005
    public static final double kDPos = 0;
    public static final double kPosErrTolerance = 1;

    public static final double max_RadPS = Math.PI/2.0;
    public static final double max_RadPSSq = Math.pow(max_RadPS,4);
  
    // max 379
    public static final double k0Lock = 0.0;    //Down
    public static final double k1Lock = 90.0;   // 1st level
    public static final double k2Lock = 280.0;  // Intake
    public static final double k3Lock = 90.0;   // 2nd level
    public static final double k4Lock = 300.0;  // 3rd level
  }

  public static class IntakeConstants {
    public static final int kMotorPort = 24;
    public static final int kHingeMotorPort = 25;

    public static final boolean kIntakeMotorInverted = true;
    public static final boolean kIntakeHingeMotorInverted = true;
    public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
    public static final IdleMode kIntakeHingeIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kIntakeCurrentLimit = 30;
    public static final int kIntakeHingeCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 0;
    public static final double kMaxOutput = 1.0;
        
    public static final double kGearBoxRatio = 1/7.0;
    public static final double kEncoderRpmToWheelRpm = kGearBoxRatio;

    public static final double kMaxVel = 1;

    public static final double kPPos = 0.05;
    public static final double kIPos = 0.01;
    public static final double kDPos = 0.0;
    public static final double kPosErrTolerance = 1;

    public static final double max_RadPS = Math.PI/0.25;
    public static final double max_RadPSSq = Math.pow(max_RadPS,2);

    public static final double k0Lock = 0.0;   // Up
    public static final double k1Lock = 110.0; // Alge
    public static final double k2Lock = 175.0; // Floor
  }

}
