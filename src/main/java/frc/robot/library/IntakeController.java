// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeController {

    private double targetPosition;
    private IntakePivotSubsystem.intakePos targetPositionEnum;

    public IntakeController(IntakePivotSubsystem.intakePos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    private double updTargetPosition(IntakePivotSubsystem.intakePos pos) {
        this.targetPositionEnum = pos;
        switch(pos) {
            case k0: 
                return IntakeConstants.k0Lock;
            case k1:
                return IntakeConstants.k1Lock;
            case k2:
                return IntakeConstants.k2Lock;
            default:
                throw new AssertionError("Illegal value: " + pos);
        }
    }

    public void setTargetPosition(IntakePivotSubsystem.intakePos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public IntakePivotSubsystem.intakePos getTargetEnum() {
        return targetPositionEnum;
    }

}
