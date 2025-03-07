package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IntakeConstants;

public class CoralSubsystem extends SubsystemBase{
    public enum coralDir{kIn, kOUt, kOff};

    private final SparkMax m_coral = new SparkMax(CoralConstants.kMotorPort, MotorType.kBrushless);
    private final SparkMax m_coralHinge = new SparkMax(CoralConstants.kMotorHingePort, MotorType.kBrushless);

    private final RelativeEncoder m_coralEncoder = m_coral.getEncoder();
    private final RelativeEncoder m_coralHingeEncoder = m_coralHinge.getEncoder();

        // Using a TrapezoidProfile PIDController to allow for smooth climbing
    private final ProfiledPIDController m_coralPIDController =
        new ProfiledPIDController(
            IntakeConstants.kPPos,
            IntakeConstants.kIPos,
            IntakeConstants.kDPos,
            new TrapezoidProfile.Constraints(
                IntakeConstants.max_RadPS * IntakeConstants.maxVoltage,
                IntakeConstants.max_RadPSSq * IntakeConstants.maxVoltage));

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
        m_coralEncoder.setPosition(0);
        m_coralHingeEncoder.setPosition(0);
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
        m_coralHinge.stopMotor();
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
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Coral Shooter Velocity", getIntakeVelocity());
        SmartDashboard.putNumber("Coral Angle", getAngle());
  }
}
