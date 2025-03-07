// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

/** Add your docs here. */
public class CoralController {

    private double targetPosition;
    private CoralSubsystem.coralPos targetPositionEnum;

    public CoralController(CoralSubsystem.coralPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    private double updTargetPosition(CoralSubsystem.coralPos pos) {
        this.targetPositionEnum = pos;
        switch(pos) {
            case k0: 
                return CoralConstants.k0Lock;
            case k1:
                return CoralConstants.k1Lock;
            case k2:
                return CoralConstants.k2Lock;
            case k3:
                return CoralConstants.k3Lock;
            default:
                throw new AssertionError("Illegal value: " + pos);
        }
    }

    public void setTargetPosition(CoralSubsystem.coralPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public CoralSubsystem.coralPos getTargetEnum() {
        return targetPositionEnum;
    }

}
