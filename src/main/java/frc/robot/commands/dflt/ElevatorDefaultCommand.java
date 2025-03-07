// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefaultCommand extends Command {
  /** Creates a new ElevatorDefaultCommand. */
  private final ElevatorSubsystem s_robotElevator;

  public ElevatorDefaultCommand(ElevatorSubsystem s_robotElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotElevator);
    this.s_robotElevator = s_robotElevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((s_robotElevator.getPosition() >= ElevatorConstants.k3Lock && RobotContainer.l_elevatorPos.getTargetEnum() == ElevatorSubsystem.elevatorPos.k3) ||
    (s_robotElevator.getPosition() <= ElevatorConstants.k0Lock && RobotContainer.l_elevatorPos.getTargetEnum() == ElevatorSubsystem.elevatorPos.k0)) {
      s_robotElevator.stopMotors();
    }
    else {
      s_robotElevator.setPosition(RobotContainer.l_elevatorPos.getTargetPosition());
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
