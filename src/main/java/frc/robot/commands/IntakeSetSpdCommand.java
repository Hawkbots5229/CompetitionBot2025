// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSetSpdCommand extends Command {
  private final IntakeSubsystem s_robotIntake;
  private final IntakeSubsystem.intakeDir direction;

  /** Creates a new IntakeSetSpdCommand. */
  public IntakeSetSpdCommand(IntakeSubsystem s_robotIntake, IntakeSubsystem.intakeDir direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotIntake);
    this.s_robotIntake = s_robotIntake;
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
        s_robotIntake.wheelsIn();
        break;
      case kOut:
        s_robotIntake.wheelsOut();
        break;
      case kOff:
        s_robotIntake.stopMotors();
        break;
      default:
        throw new AssertionError("Illegal value: " + direction);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotIntake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
