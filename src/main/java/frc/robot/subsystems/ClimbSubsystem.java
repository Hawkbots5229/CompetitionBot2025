package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    private final SparkMax m_climb =
        new SparkMax(ClimbConstants.kMotorPort, MotorType.kBrushless);
    
    private final RelativeEncoder m_climbEncoder = m_climb.getEncoder();

    private SparkMaxConfig m_climbConfig;

    public ClimbSubsystem() {
        m_climbConfig.inverted(ClimbConstants.kClimbMotorInverted);
        m_climbConfig.idleMode(ClimbConstants.kIdleMode);
        m_climbConfig.voltageCompensation(ClimbConstants.maxVoltage);
        m_climbConfig.smartCurrentLimit(ClimbConstants.kCurrentLimit);

        m_climb.configure(m_climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        resetEncoders();
    }

    /**Sets the encoder positions to 0
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.relativeEncoder.setPosition()
     * 
     */
    public void resetEncoders() {
        m_climbEncoder.setPosition(0);
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
        return m_climbEncoder.getPosition() * Math.PI * 2;
    }

    /**Sets the speed of the motor
     * 
     * @return Void
     * @param output the speed to set.  Should be between -1 and 1
     * @implNote com.revrobotics.spark.SparkBase.set()
     * 
     */
    public void setTargetOutput(double output) {
        m_climb.set(output);
    }

    /**Stops the motor
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.spark.SparkBase.stopMotor()
     * 
     */
    public void stopMotor() {
        m_climb.stopMotor();
    }

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        SmartDashboard.putNumber("Climb Angle", getAngle());
    }
}
