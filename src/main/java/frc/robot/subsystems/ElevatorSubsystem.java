package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

    public enum elevatorPos{k0, k1, k2, k3};

    private final SparkMax m_left =
        new SparkMax(ElevatorConstants.kLeftMotorPort, MotorType.kBrushless);
    private final SparkMax m_right =
        new SparkMax(ElevatorConstants.kRigthMotorPort, MotorType.kBrushless);

    private final RelativeEncoder m_leftEncoder = m_left.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_right.getEncoder();

    private final ProfiledPIDController m_liftingPIDController =
        new ProfiledPIDController(
            ElevatorConstants.kPPos,
            ElevatorConstants.kIPos,
            ElevatorConstants.kDPos,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVel * ElevatorConstants.maxVoltage,
                ElevatorConstants.kMaxAcc * ElevatorConstants.maxVoltage));

    public ElevatorSubsystem() {
        m_left.configure(
            new SparkMaxConfig().
                inverted(ElevatorConstants.kLeftMoterInverted).
                idleMode(ElevatorConstants.kIdleMode).
                voltageCompensation(ElevatorConstants.maxVoltage).
                smartCurrentLimit(ElevatorConstants.kCurrentLimit), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters);

        m_right.configure(
            new SparkMaxConfig().
                inverted(ElevatorConstants.kRightMotorInverted).
                idleMode(ElevatorConstants.kIdleMode).
                voltageCompensation(ElevatorConstants.maxVoltage).
                smartCurrentLimit(ElevatorConstants.kCurrentLimit), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters);

        resetEncoders();

        m_liftingPIDController.setTolerance(ElevatorConstants.kPosErrTolerance);
    }

    /**Sets the position of the encoders to 0
     * 
     * @return Void
     * @param None
     * @implNote com.revrobotics.RelativeEncoder.setPosition()
     * 
     */
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    /**Returns the height of the arm
     * 
     * @return height of the arm
     * @param None
     * @implNote com.revrobotics.RelativeEncoder.getPosition()
     * 
     */
    public double getPosition() {
        return m_leftEncoder.getPosition();
    }

    /**Sets the target height of the elevator
     * Uses PID control to determine the motor speed required to reach that height
     * 
     * @return None
     * @param tarHeight The target height of the elevator
     * @implNote edu.wpi.first.math.controller.ProfiledPIDController()
     * @implNote getPosition()
     * @implNote com.revrobotics.spark.SparkBase.set()
     * 
     */
    public void setPosition(double tarHeight) {
        double output = m_liftingPIDController.calculate(getPosition(), tarHeight);
        m_left.set(output);
        m_right.set(output);
    }

    /**Stops the motors
     * 
     * @return None
     * @param Void
     * @implNote com.revrobotics.spark.SparkBase.stopMotor()
     * 
     */
    public void stopMotors() {
        m_left.stopMotor();
        m_right.stopMotor();
    }

    @Override
    public void periodic() {
        //this method will be called once per scheduler run
        SmartDashboard.putNumber("Arm Height", getPosition());
    }
}
