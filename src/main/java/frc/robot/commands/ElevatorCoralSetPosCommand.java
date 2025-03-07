// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorCoralSetPosCommand extends InstantCommand {
  private final ElevatorSubsystem.elevatorPos posElevator;
  private final CoralSubsystem.coralPos posCoral;

  public ElevatorCoralSetPosCommand(ElevatorSubsystem.elevatorPos posElevator, CoralSubsystem.coralPos posCoral) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.posElevator = posElevator;
    this.posCoral = posCoral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.l_elevatorPos.setTargetPosition(posElevator);
    RobotContainer.l_coralPos.setTargetPosition(posCoral);
  }
}
