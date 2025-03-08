package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    public enum intakeDir{kIn, kOut, kOff};


    private final SparkMax m_intake = new SparkMax(IntakeConstants.kMotorPort, MotorType.kBrushless);
    

    private final RelativeEncoder m_intakeEncoder = m_intake.getEncoder();
    

    

    public IntakeSubsystem() {
        m_intake.configure(
            new SparkMaxConfig().
                inverted(IntakeConstants.kIntakeMotorInverted).
                idleMode(IntakeConstants.kIntakeIdleMode).
                secondaryCurrentLimit(IntakeConstants.kIntakeCurrentLimit).
                openLoopRampRate(IntakeConstants.kOpenLoopRampRate).
                voltageCompensation(IntakeConstants.maxVoltage)
            , ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        
    }

    public void resetEncoders() {
        m_intakeEncoder.setPosition(0);
    }

    public void setTargetOutput(double output) {
        m_intake.set(output);
    }

    public void wheelsIn() {
        setTargetOutput(IntakeConstants.kMaxOutput);
    }

    public void wheelsOut() {
        setTargetOutput(-IntakeConstants.kMaxOutput);
    }

    public void stopMotors() {
        m_intake.stopMotor();
    }

    public double getIntakeVelocity() {
        return m_intakeEncoder.getVelocity();
    }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
        
  }
}
