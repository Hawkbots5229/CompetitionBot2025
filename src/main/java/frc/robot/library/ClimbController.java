// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

/** Add your docs here. */
public class ClimbController {

    private double targetPosition;
    private ClimbSubsystem.climbPos targetPositionEnum;

    public ClimbController(ClimbSubsystem.climbPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    private double updTargetPosition(ClimbSubsystem.climbPos pos) {
        this.targetPositionEnum = pos;
        switch(pos) {
            case k0: 
                return ClimbConstants.k0Lock;
            case k1:
                return ClimbConstants.k1Lock;
            default:
                throw new AssertionError("Illegal value: " + pos);
        }
    }

    public void setTargetPosition(ClimbSubsystem.climbPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public ClimbSubsystem.climbPos getTargetEnum() {
        return targetPositionEnum;
    }

}
