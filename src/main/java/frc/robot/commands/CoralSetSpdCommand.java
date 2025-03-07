// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralSetSpdCommand extends Command {
  private final CoralSubsystem s_robotCoral;
  private final CoralSubsystem.coralDir direction;

  /** Creates a new CoralSetSpdCommand. */
  public CoralSetSpdCommand(CoralSubsystem s_robotCoral, CoralSubsystem.coralDir direction) {
    addRequirements(s_robotCoral);
    this.s_robotCoral = s_robotCoral;
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (direction) {
      case kIn:
        s_robotCoral.wheelsIn();
        break;
      case kOUt:
        s_robotCoral.wheelsOut();
        break;
      case kOff:
        s_robotCoral.stopMotors();
        break;
      default:
        throw new AssertionError("Illegal value: " + direction);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotCoral.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
