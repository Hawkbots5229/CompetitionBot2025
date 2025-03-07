package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase{
    public enum coralDir{kIn, kOUt, kOff};

    private final SparkMax m_coral = new SparkMax(CoralConstants.kMotorPort, MotorType.kBrushless);

    private final RelativeEncoder m_coralEncoder = m_coral.getEncoder();

    public CoralSubsystem() {
        m_coral.configure(
            new SparkMaxConfig().
                inverted(CoralConstants.kCoralMotorInverted).
                idleMode(CoralConstants.kCoralIdleMode).
                openLoopRampRate(CoralConstants.kOpenLoopRampRate).
                voltageCompensation(CoralConstants.maxVoltage).
                secondaryCurrentLimit(CoralConstants.kCoralCurrentLimit), 
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    /**Sets the encoder positions to 0
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.relativeEncoder.setPosition()
     * 
     */
    public void resetEncoders() {
        m_coralEncoder.setPosition(0);
    }

    /** Sets the desired speed of the left and right intake motors.
   * 
   * @return Void
   * @param output The speed to set. Value should be between -1.0 and 1.0.
   * @implNote com.revrobotics.spark.SparkBase.set()
   * 
   */
    public void setTargetOutput(double output) {
        m_coral.set(output);
    }

    /** Spins intake backwards.
   * 
   * @return Void
   * @param None
   * @implNote setTargetOutput()
   * @implNote IntakeConstants.kMaxOutput
   * 
   */
    public void wheelsIn() {
        setTargetOutput(CoralConstants.kMaxOutput);
    }

    /** Spins intake forward.
   * 
   * @return Void
   * @param None
   * @implNote setTargetOutput()
   * @implNote IntakeConstants.kMaxOutput
   * 
   */
    public void wheelsOut() {
        setTargetOutput(-CoralConstants.kMaxOutput);
    }

    /** Stops the motors.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.spark.SparkBase.stopMotor()
   * 
   */
    public void stopMotors() {
        m_coral.stopMotor();
    }
    
    /** Calculates the average motor velocities.
   * 
   * @return Velocity of intake motors (rev/sec)
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.getVelocity()
   * 
   */
    public double getIntakeVelocity() {
        return m_coralEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Coral Shooter Velocity", getIntakeVelocity());
  }
}
