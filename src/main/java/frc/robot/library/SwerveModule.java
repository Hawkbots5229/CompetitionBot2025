// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.DriveConstants;;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final CANcoder m_turningEncoderAbs;
  private final SwerveData m_swerveData;
  //private final VoltageOut vo_driveMotor;
  //private final VoltageOut vo_turningMotor;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.steerKp,
          SwerveConstants.steerKi,
          SwerveConstants.steerKd,
          new TrapezoidProfile.Constraints(
            SwerveConstants.steerMax_RadPS * DriveConstants.maxVoltage,
            SwerveConstants.steerMax_RadPSSq * DriveConstants.maxVoltage));

  /** Constructs a SwerveModule.
   *
   * @param swerveData Data for swever module
   */
  public SwerveModule(SwerveData swerveData) {


    m_swerveData = swerveData;
    m_driveMotor = new TalonFX(swerveData.driveCANId, DriveConstants.kCanBus);
    m_turningMotor = new TalonFX(swerveData.steerCANId, DriveConstants.kCanBus);

    //vo_driveMotor = new VoltageOut(0.0);
    //vo_turningMotor = new VoltageOut(0.0);

    m_turningEncoderAbs = new CANcoder(swerveData.encoderCANId, DriveConstants.kCanBus);
    CANcoderConfiguration cfg_turningEncoderAbs = new CANcoderConfiguration();
    cfg_turningEncoderAbs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0; 
    cfg_turningEncoderAbs.MagnetSensor.SensorDirection = 
      swerveData.steerEncoderInvert
      ? SensorDirectionValue.Clockwise_Positive
      : SensorDirectionValue.CounterClockwise_Positive;
    cfg_turningEncoderAbs.MagnetSensor.MagnetOffset = swerveData.steerAngleOffset;
    //cfg_turningEncoderAbs.MagnetSensor.MagnetOffset = 0.0;
    m_turningEncoderAbs.getConfigurator().apply(cfg_turningEncoderAbs);
  

    TalonFXConfiguration cfg_driveMotor = new TalonFXConfiguration();
    cfg_driveMotor.MotorOutput.Inverted = 
      swerveData.driveMotorInvert
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;

    m_driveMotor.setNeutralMode(DriveConstants.driveMode);
    m_driveMotor.getConfigurator().apply(cfg_driveMotor);
  
    TalonFXConfiguration cfg_turningMotor = new TalonFXConfiguration();
    cfg_turningMotor.MotorOutput.Inverted = 
      swerveData.steerMotorInvert
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;

    //m_turningMotor.setNeutralMode(DriveConstants.turnMode);
    m_turningMotor.getConfigurator().apply(cfg_turningMotor);
  
    m_turningMotor.setNeutralMode(DriveConstants.turnMode);

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous. 
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(0.01);
  }

  /** Gets the absolute position of the swerve wheels
   * 
   * @return Angle (Radians)
   * @param None
   * @implNote com.ctre.phoenix.sensors.CANcoder.getAbsolutePosition()
   */
  public double getSwerveAngle(){
    
    //return (m_turningEncoderAbs.getAbsolutePosition().getValueAsDouble()*2.0*Math.PI);
    return (m_turningEncoderAbs.getAbsolutePosition().getValueAsDouble());
  }

  /** Gets the relative position of the swerve wheels.
   * 
   * @return Angle (Revolutions)
   * @param None
   * @implNote com.ctre.phoenix.sensors.CANcoder.getAbsolutePosition()
   */
  public double getSteerMotorAngle(){
    return m_turningMotor.getPosition().getValueAsDouble();
  }

  /** Gets the angle of the drive wheel
   * 
   * @return Angle (Radians)
   * @param None
   * @implNote getSteerMotorAngle
   * @implNote SwerveConstants.steer_CntsPRad
   */
  public double getSteerAngle(){
      return getSteerMotorAngle() / SwerveConstants.steer_RevPRad;   
  }

  /** Sets the relative steering motor encoders to the absolute position or
   *  0 if there is no absolute position.
   * 
   * @return Void
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.setSelectedSensorPosition()
   * @implNote getSwerveAngle()
   * @implNote SwerveConstants.steer_CntsPRad
   */
  public void resetSteerSensors(){
    //if(m_swerveData.useAbsEnc) {
    //  m_turningMotor.setPosition((getSwerveAngle() - Math.toRadians(m_swerveData.steerAngleOffset))*SwerveConstants.steer_RevPRad);
    //}
    //else {
    //  m_turningMotor.setPosition(0);
    //}
    System.out.println(m_swerveData.name + "inti value: " + getSwerveAngle());
     m_turningMotor.setPosition(getSwerveAngle()*SwerveConstants.steerRatio);
  }

  /** Resets the drive motor encoders to a position of 0.
   * 
   * @return Void
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.setSelectedSensorPosition()
   */
  public void resetDriveEncoders(){
    m_driveMotor.setPosition(0);
  }

  /** Resets both the relative drive and steering motor encoders
   * 
   * @return Void
   * @param None
   * @implNote resetDriveEncoders()
   * @implNote resetSteerSensors()
   */
  public void resetEncoders() {
    resetDriveEncoders();
    resetSteerSensors();
  }

  /** Gets the distance driven in meters
   * 
   * @return Distance (meters)
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.getSelectedSensorPosition()
   * @implNote SwerveConstants.driveDistanceCntsPMeter
   */
  public double getDriveDistanceMeters(){
    final double dis = m_driveMotor.getPosition().getValueAsDouble();
    final double meters = dis / SwerveConstants.driveDistanceCntsPMeter;
    return Math.abs(meters);
  }

  /** Gets the distance driven in inches
   * 
   * @return Distance (inches)
   * @param None
   * @implNote getDriveDistanceMeters()
   * @implNote DriveConstants.MetersPerInch
   */
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() / DriveConstants.MetersPerInch;
  }

  /** Gets the velocity of the drive wheel
   * 
   * @return Velocity (meters/sec)
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.getSelectedSensorVelocity()
   * @implNote SwerveConstants.driveRawVelocityToMPS
   */
  public double getDriveVelocity(){
    double vel1 = m_driveMotor.getVelocity().getValueAsDouble();
    double velocity = vel1 / SwerveConstants.driveRawVelocityToMPS;
    return velocity;
  }

   /** Stop drive and steering motors
   * 
   * @return Void
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.TalonFX.stopMotor()
   * @implNote com.revrobotics.CANSparkMax.stopMotor()
   */
  public void stopMotors(){
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }

  /** Returns the current position of the swerve module as a swerve module position structure.
   *
   * @return Swerve Module Position
   * @param None
   * @implNote edu.wpi.first.math.kinematics.SwerveModulePosition()
   * @implNote edu.wpi.first.math.geometry.Rotation2d()
   * @implNote getDriveDistanceMeters()
   * @implNote getSteerAngle()
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDriveDistanceMeters(), new Rotation2d(getSteerAngle()));
  }

  /** Returns the current state of the swerve module as a swerve module state structure.
   * 
   * @return Swerve Module State
   * @param None
   * @implNote edu.wpi.first.math.kinematics.SwerveModuleState()
   * @implNote edu.wpi.first.math.geometry.Rotation2d()
   * @implNote getDriveVelocity()
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  /** Sets the desired state for the module.
   * 
   * @return Void
   * @param desiredState Desired state with speed and angle.
   * @param optimize Whether to optimize steering
   * @param disableDrive Whether to disable drive for testing
   * @param disableSteer Whether to disable steering for testing
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean optimize, boolean disableDrive, boolean disableSteer) {

    double steerOutput = 0;

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state;
    if(optimize){
        state = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerAngle()));
    }else{
        state = desiredState;
    }

    if(!disableDrive){
      m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.maxSpeed);
    }
    // Calculate the turning motor output from the turning PID controller.
    if(!disableSteer){
        steerOutput = m_turningPIDController.calculate(getSteerAngle(), state.angle.getRadians());
        m_turningMotor.set(steerOutput);
    }
  }

  /** Puts swerve data to the dashboard
   * 
   * @return Void
   * @param None
   */
  public void sendData(){
    
    //SmartDashboard.putNumber(m_swerveData.name + "SteerMotorAngle", getSteerMotorAngle());
    SmartDashboard.putNumber(m_swerveData.name + "CANcoderAngle", getSwerveAngle());
    //SmartDashboard.putNumber(m_swerveData.name + "WheelAngle", Math.toDegrees(getSteerAngle()));
    //SmartDashboard.putNumber(m_swerveData.name + "DriveDistance", getDriveDistanceInches());
    //SmartDashboard.putNumber(m_swerveData.name + "DriveVelocity", getDriveVelocity());
    
  }
}
