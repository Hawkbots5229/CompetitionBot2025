// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousElevatorSetPos extends Command {
  private final ElevatorSubsystem s_robotElevator;
  private final CoralPivotSubsystem s_coralPivot;
  private final ElevatorSubsystem.elevatorPos elevatorPos;
  private final CoralPivotSubsystem.coralPos coralPos;

  public AutonomousElevatorSetPos(ElevatorSubsystem s_robotElevator, CoralPivotSubsystem s_coralPivot, ElevatorSubsystem.elevatorPos elevatorPos, CoralPivotSubsystem.coralPos coralPos) {
    addRequirements(s_robotElevator);
    this.s_robotElevator = s_robotElevator;
    this.s_coralPivot = s_coralPivot;
    this.elevatorPos = elevatorPos;
    this.coralPos = coralPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.l_elevatorPos.setTargetPosition(elevatorPos);
    RobotContainer.l_coralPos.setTargetPosition(coralPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotElevator.setPosition(RobotContainer.l_elevatorPos.getTargetPosition());
    s_coralPivot.setPosition(RobotContainer.l_coralPos.getTargetPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotElevator.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.l_elevatorPos.getTargetPosition() - s_robotElevator.getPosition()) <= .005;
  }
}
