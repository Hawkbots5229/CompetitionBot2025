// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorController {

    private double targetPosition;
    private ElevatorSubsystem.elevatorPos targetPositionEnum;

    public ElevatorController(ElevatorSubsystem.elevatorPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    private double updTargetPosition(ElevatorSubsystem.elevatorPos pos) {
        this.targetPositionEnum = pos;
        switch(pos) {
            case k0: 
                return ElevatorConstants.k0Lock;
            case k1:
                return ElevatorConstants.k1Lock;
            case k2:
                return ElevatorConstants.k2Lock;
            case k3:
                return ElevatorConstants.k3Lock;
            default:
                throw new AssertionError("Illegal value: " + pos);
        }
    }

    public void setTargetPosition(ElevatorSubsystem.elevatorPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public ElevatorSubsystem.elevatorPos getTargetEnum() {
        return targetPositionEnum;
    }

}
