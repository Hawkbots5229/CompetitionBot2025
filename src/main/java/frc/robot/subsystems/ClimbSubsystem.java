package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;


public class ClimbSubsystem extends SubsystemBase{

    public enum climbPos{k0, k1};

    private final TalonFX m_climbMotor = new TalonFX(ClimbConstants.kMotorPort, ClimbConstants.kCanBus);

    // Using a TrapezoidProfile PIDController to allow for smooth climbing
    private final ProfiledPIDController m_climbPIDController =
        new ProfiledPIDController(
            ClimbConstants.kPPos,
            ClimbConstants.kIPos,
            ClimbConstants.kDPos,
            new TrapezoidProfile.Constraints(
                ClimbConstants.climbMax_RadPS * ClimbConstants.maxVoltage,
                ClimbConstants.climbMax_RadPSSq * ClimbConstants.maxVoltage));

    public ClimbSubsystem() {
        TalonFXConfiguration cfg_climbMotor = new TalonFXConfiguration();
        cfg_climbMotor.MotorOutput.Inverted = 
        ClimbConstants.kClimbMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

        m_climbMotor.setNeutralMode(ClimbConstants.kIdleMode);
        m_climbMotor.getConfigurator().apply(cfg_climbMotor);

        resetEncoders();

        m_climbPIDController.setTolerance(ClimbConstants.kPosErrTolerance);
    }

    /**Sets the encoder positions to 0
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.relativeEncoder.setPosition()
     * 
     */
    public void resetEncoders() {
        m_climbMotor.setPosition(0);
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
        return m_climbMotor.getPosition().getValueAsDouble() * Math.PI * 2;
    }

    /**Sets the speed of the motor
     * 
     * @return Void
     * @param output the speed to set.  Should be between -1 and 1
     * @implNote com.revrobotics.spark.SparkBase.set()
     * 
     */
    public void setTargetOutput(double output) {
        m_climbMotor.set(output);
    }

    /**Stops the motor
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.spark.SparkBase.stopMotor()
     * 
     */
    public void stopMotor() {
        m_climbMotor.stopMotor();
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
        double output = m_climbPIDController.calculate(getAngle(), tarAngle);
        m_climbMotor.set(output);
    }

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        SmartDashboard.putNumber("Climb Angle", getAngle());
    }
}
