// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousIntakeSetSpd extends Command {
  private final IntakeSubsystem s_robotIntake;
  private final Timer tmr = new Timer();
  private final double speed;
  private final double time;
  public AutonomousIntakeSetSpd(IntakeSubsystem s_robotIntake, double speed, double time) {
    addRequirements(s_robotIntake);
    this.s_robotIntake = s_robotIntake;
    this.speed = speed;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tmr.reset();
    tmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotIntake.setTargetOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tmr.get() > time;
  }
}
