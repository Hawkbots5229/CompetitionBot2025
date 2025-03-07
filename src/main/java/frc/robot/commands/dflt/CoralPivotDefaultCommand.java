// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPivotDefaultCommand extends Command {
  /** Creates a new CoralDefaultCommand. */
  private final CoralPivotSubsystem s_robotCoral;

  public CoralPivotDefaultCommand(CoralPivotSubsystem s_robotCoral) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotCoral);
    this.s_robotCoral = s_robotCoral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((s_robotCoral.getAngle() >= CoralConstants.k2Lock && RobotContainer.l_coralPos.getTargetEnum() == CoralPivotSubsystem.coralPos.k2) ||
    (s_robotCoral.getAngle() <= CoralConstants.k0Lock && RobotContainer.l_coralPos.getTargetEnum() == CoralPivotSubsystem.coralPos.k0)) {
      s_robotCoral.stopHingeMotors();
    }
    else {
      s_robotCoral.setPosition(RobotContainer.l_coralPos.getTargetPosition());
    }
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
