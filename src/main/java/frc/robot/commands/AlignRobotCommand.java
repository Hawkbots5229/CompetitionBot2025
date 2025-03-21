// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRobotCommand extends Command {
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

  PIDController m_rotationPIDController = new PIDController(DriveConstants.rotKp, DriveConstants.rotKi, DriveConstants.rotKd);
  DrivetrainSubsystem drive;

  public AlignRobotCommand(DrivetrainSubsystem s_robotDrive) {
    addRequirements(s_robotDrive);
    drive = s_robotDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationPIDController.setTolerance(DriveConstants.rotToleranceDeg, DriveConstants.rotToleranceVel);
    m_rotationPIDController.setIntegratorRange(0, .5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = 0;

    //TODO    
    if (drive.targetVisible()) {
      rot = drive.getTargetYaw() * DriveConstants.maxAngularSpeed;
    }

    drive.drive(0, 0, rot * DriveConstants.rotationScaleMin, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getTargetYaw() == 0;
  }
}
