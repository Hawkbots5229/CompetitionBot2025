// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IntakeConstants;

public class CoralPivotSubsystem extends SubsystemBase {
  public enum coralPos{k0, k1, k2, k3, k4};

  private final SparkMax m_coralHinge = new SparkMax(CoralConstants.kMotorHingePort, MotorType.kBrushless);
  private final RelativeEncoder m_coralHingeEncoder = m_coralHinge.getEncoder();

  private final ProfiledPIDController m_coralPIDController =
      new ProfiledPIDController(
          IntakeConstants.kPPos,
          IntakeConstants.kIPos,
          IntakeConstants.kDPos,
          new TrapezoidProfile.Constraints(
              IntakeConstants.max_RadPS * IntakeConstants.maxVoltage,
              IntakeConstants.max_RadPSSq * IntakeConstants.maxVoltage));

  public CoralPivotSubsystem() {
    m_coralHinge.configure(
            new SparkMaxConfig().
                inverted(CoralConstants.kCoralHingeMotorInverted).
                idleMode(CoralConstants.kCoralHingeIdleMode).
                openLoopRampRate(CoralConstants.kOpenLoopRampRate).
                voltageCompensation(CoralConstants.maxVoltage).
                secondaryCurrentLimit(CoralConstants.kCoralHingeCurrentLimit), 
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters);

        resetEncoders();
        
        m_coralPIDController.setTolerance(ClimbConstants.kPosErrTolerance);
  }

  /**Sets the encoder positions to 0
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.relativeEncoder.setPosition()
     * 
     */
    public void resetEncoders() {
      m_coralHingeEncoder.setPosition(0);
  }

  /** Stops the motors.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.spark.SparkBase.stopMotor()
   * 
   */
  public void stopHingeMotors() {
    m_coralHinge.stopMotor();
  }

  /**Gets the angle of the motor in radians
     * 
     * @return current motor angle (radians)
     * @param None
     * @implNote com.revrobotics.RelativeEncoder.getPosition()
     * @implNote Math.PI
     * 
     */
    public double getAngle() {
      return m_coralHingeEncoder.getPosition() * Math.PI * 2;
  }

  /**Sets the speed of the motor
     * 
     * @return Void
     * @param output the speed to set.  Should be between -1 and 1
     * @implNote com.revrobotics.spark.SparkBase.set()
     * 
     */
    public void setTargetHingeOutput(double output) {
      m_coralHinge.set(output);
  }

  /**Sets the target angle of the climber
   * Uses PID control to determine the motor speed required to reach that angle
   * 
   * @return None
   * @param tarAngle The target anggle
   * @implNote edu.wpi.first.math.controller.ProfiledPIDController()
   * @implNote getPosition()
   * 
   */
  public void setPosition(double tarAngle) {
      double output = m_coralPIDController.calculate(getAngle(), tarAngle);
      m_coralHinge.set(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Angle", getAngle());
  }
}
