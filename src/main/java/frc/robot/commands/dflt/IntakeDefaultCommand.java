// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeDefaultCommand extends Command {
  /** Creates a new CoralDefaultCommand. */
  private final IntakeSubsystem s_robotIntake;

  public IntakeDefaultCommand(IntakeSubsystem s_robotIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotIntake);
    this.s_robotIntake = s_robotIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((s_robotIntake.getAngle() >= IntakeConstants.k2Lock && RobotContainer.l_intakePos.getTargetEnum() == IntakeSubsystem.intakePos.k2) ||
    (s_robotIntake.getAngle() <= IntakeConstants.k0Lock && RobotContainer.l_intakePos.getTargetEnum() == IntakeSubsystem.intakePos.k0)) {
      s_robotIntake.stopHingeMotors();
    }
    else {
      s_robotIntake.setPosition(RobotContainer.l_intakePos.getTargetPosition());
    }
    s_robotIntake.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
