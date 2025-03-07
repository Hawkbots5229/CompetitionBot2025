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
import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase {
  public enum intakePos{k0, k1, k2};

  private final SparkMax m_intakeHinge = new SparkMax(IntakeConstants.kHingeMotorPort, MotorType.kBrushless);

  private final RelativeEncoder m_intakeHingeEncoder = m_intakeHinge.getEncoder();

  // Using a TrapezoidProfile PIDController to allow for smooth climbing
    private final ProfiledPIDController m_intakePIDController =
        new ProfiledPIDController(
            IntakeConstants.kPPos,
            IntakeConstants.kIPos,
            IntakeConstants.kDPos,
            new TrapezoidProfile.Constraints(
                IntakeConstants.max_RadPS * IntakeConstants.maxVoltage,
                IntakeConstants.max_RadPSSq * IntakeConstants.maxVoltage));

  public IntakePivotSubsystem() {
    m_intakeHinge.configure(
            new SparkMaxConfig().
                inverted(IntakeConstants.kIntakeHingeMotorInverted).
                idleMode(IntakeConstants.kIntakeHingeIdleMode).
                secondaryCurrentLimit(IntakeConstants.kIntakeHingeCurrentLimit).
                openLoopRampRate(IntakeConstants.kOpenLoopRampRate).
                voltageCompensation(IntakeConstants.maxVoltage)
            , ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        resetEncoders();

        m_intakePIDController.setTolerance(ClimbConstants.kPosErrTolerance);
  }

  public void resetEncoders() {
    m_intakeHingeEncoder.setPosition(0);
  }

  public void stopHingeMotors() {
    m_intakeHinge.stopMotor();
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
      return m_intakeHingeEncoder.getPosition() * Math.PI * 2;
  }

  /**Sets the speed of the motor
   * 
   * @return Void
   * @param output the speed to set.  Should be between -1 and 1
   * @implNote com.revrobotics.spark.SparkBase.set()
   * 
   */
  public void setTargetHingeOutput(double output) {
      m_intakeHinge.set(output);
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
      double output = m_intakePIDController.calculate(getAngle(), tarAngle);
      m_intakeHinge.set(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Angle", getAngle());
  }
}
