// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

/** Add your docs here. */
public class SwerveData {

    public String name = "";
    public int steerCANId = 0;
    public int driveCANId = 0;
    public int encoderCANId = 0;
    public Boolean driveMotorInvert = false;
    public Boolean steerMotorInvert = false;
    public Boolean steerEncoderInvert = false;
    public double steerAngleOffset = 0;
    public Boolean useAbsEnc = false;

    /** Construct to hold swerve data
     * 
     * @param name                 String name of this device
     * @param driveCANId           Drive CAN Id
     * @param driveMotorInvert     Invert Drive of Motor
     * @param steerCANId           Steer CAN Id
     * @param steerMotorInvert     Invert Steer of Motor
     * @param steerEncoderInvert   Invert Steer Encoder
     * @param encoderCANId         Encoder CAN Id
     * @param encoderSernsorDir    Encoder Sensor Direction
     * @param steerAnalogOffset    Steer Analog Offset in Degrees.
     */
    public SwerveData(String name, int driveCANId,  Boolean driveMotorInvert, int steerCANId, Boolean steerMotorInvert, 
                                   int encoderCANId, Boolean steerEncoderInvert, double steerAnalogOffset, Boolean useAbsEnc ){

        this.name = name;
        this.driveCANId = driveCANId;
        this.steerCANId = steerCANId;
        this.encoderCANId = encoderCANId;
        this.driveMotorInvert = driveMotorInvert;
        this.steerMotorInvert = steerMotorInvert;
        this.steerEncoderInvert = steerEncoderInvert;
        this.steerAngleOffset = steerAnalogOffset;
        this.useAbsEnc = useAbsEnc;
    }
}