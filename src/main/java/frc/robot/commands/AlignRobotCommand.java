// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRobotCommand extends Command {
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(8, 8);

  private final int targetTag;
  private static final Transform3d tagToGoal =
    new Transform3d(new Translation3d(1.5, 0.0, 0.0),
    new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem drive;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(DriveConstants.rotKp, DriveConstants.rotKi, DriveConstants.rotKd, rotConstraints);
  
  private PhotonTrackedTarget lastTarget;
  
  public AlignRobotCommand(PhotonCamera photonCamera, DrivetrainSubsystem drive, Supplier<Pose2d> poseProvider, int targetTag) {
    this.photonCamera = photonCamera;
    this.drive = drive;
    this.poseProvider = poseProvider;
    this.targetTag = targetTag;

    xController.setTolerance(.2);
    yController.setTolerance(.2);
    rotController.setTolerance(Units.degreesToRadians(3));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   lastTarget = null;
   var robotPose = poseProvider.get();
   rotController.reset(robotPose.getRotation().getRadians());
   xController.reset(robotPose.getX());
   yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose =
      new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

    var photonRes = photonCamera.getLatestResult();
    if(photonRes.hasTargets()) {
      var targetOpt = photonRes.getTargets().stream()
        .filter(t -> t.getFiducialId() == targetTag)
        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
        .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();

        lastTarget = target;

        Pose3d cameraPose = robotPose.transformBy(new Transform3d(.165, .165, 0, new Rotation3d()));

        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        rotController.setGoal(goalPose.getRotation().getRadians());
      }
    }

    if (lastTarget == null) {
      drive.stopMotors();
    } else {
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var rotSpeed = rotController.calculate(robotPose2d.getRotation().getRadians());
      if (rotController.atGoal()) {
        rotSpeed = 0;
      }

      drive.drive(xSpeed, ySpeed, rotSpeed, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
