// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignRobotCommand;
import frc.robot.commands.ClimberSetPosCommand;
import frc.robot.commands.ClimberSetSpdCommand;
import frc.robot.commands.CoralSetAngleCommand;
import frc.robot.commands.CoralSetPosCommand;
import frc.robot.commands.CoralSetSpdCommand;
import frc.robot.commands.DriveTrainOrientation;
import frc.robot.commands.ElevatorCoralSetPosCommand;
import frc.robot.commands.ElevatorSetPosCommand;
import frc.robot.commands.IntakeSetAngleCommand;
import frc.robot.commands.IntakeSetPosCommand;
import frc.robot.commands.IntakeSetSpdCommand;
import frc.robot.commands.auton.AutonomousCoralSetSpd;
import frc.robot.commands.auton.AutonomousElevatorSetPos;
import frc.robot.commands.dflt.ClimbDefaultCommand;
import frc.robot.commands.dflt.CoralDefaultCommand;
import frc.robot.commands.dflt.CoralPivotDefaultCommand;
import frc.robot.commands.dflt.DriveTrainDefaultCommand;
import frc.robot.commands.dflt.ElevatorDefaultCommand;
import frc.robot.commands.dflt.IntakeDefaultCommand;
import frc.robot.commands.dflt.IntakePivotDefaultCommand;
import frc.robot.library.ElevatorController;
import frc.robot.library.IntakeController;
import frc.robot.library.CoralController;
import frc.robot.library.ClimbController;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CoralPivotSubsystem.coralPos;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPos;
import frc.robot.subsystems.IntakePivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();
  private DriveTrainDefaultCommand driveTrainDefaultCommand = new DriveTrainDefaultCommand(m_robotDrive);
  public static ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  public static ElevatorController l_elevatorPos = new ElevatorController(ElevatorSubsystem.elevatorPos.k0);
  private ElevatorDefaultCommand elevatorDefaultCommand = new ElevatorDefaultCommand(m_robotElevator);
  public static CoralSubsystem m_robotCoral = new CoralSubsystem();
  public static CoralPivotSubsystem m_coralPivot = new CoralPivotSubsystem();
  private CoralDefaultCommand coralDefaultCommand = new CoralDefaultCommand(m_robotCoral);
  private CoralPivotDefaultCommand coralPivotDefaultCommand = new CoralPivotDefaultCommand(m_coralPivot);
  public static CoralController l_coralPos = new CoralController(CoralPivotSubsystem.coralPos.k0);
  public static IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  public static IntakePivotSubsystem m_IntakePivot = new IntakePivotSubsystem();
  private IntakeDefaultCommand intakeDefaultCommand = new IntakeDefaultCommand(m_robotIntake);
  private IntakePivotDefaultCommand intakePivotDefaultCommand = new IntakePivotDefaultCommand(m_IntakePivot);
  public static IntakeController l_intakePos = new IntakeController(IntakePivotSubsystem.intakePos.k0);
  public static ClimbSubsystem m_robotClimb = new ClimbSubsystem();
  public static ClimbController l_climbPos = new ClimbController(ClimbSubsystem.climbPos.k0);
  public ClimbDefaultCommand climbDefualtCommand = new ClimbDefaultCommand(m_robotClimb);

  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController m_mechController = new XboxController(OIConstants.kMechControllerPort);

  private final PhotonCamera rightCamera = new PhotonCamera("Right Camera");
  private final PhotonCamera leftCamera = new PhotonCamera("Left Camera");

  private final SendableChooser<Command> sc_autonSelect;

  public RobotContainer() {
    //Configure button bindings
    configureBindings();

    //create camera servers
    CameraServer.startAutomaticCapture("Elevator Camera", 0);

    sc_autonSelect = AutoBuilder.buildAutoChooser();

    m_robotDrive.setDefaultCommand(driveTrainDefaultCommand);
    m_robotElevator.setDefaultCommand(elevatorDefaultCommand);
    m_robotCoral.setDefaultCommand(coralDefaultCommand);
    m_coralPivot.setDefaultCommand(coralPivotDefaultCommand);
    m_robotIntake.setDefaultCommand(intakeDefaultCommand);
    m_IntakePivot.setDefaultCommand(intakePivotDefaultCommand);
    m_robotClimb.setDefaultCommand(climbDefualtCommand);

    SmartDashboard.putData("Auton Selection", sc_autonSelect);

    NamedCommands.registerCommand("Coral Set Speed", new AutonomousCoralSetSpd(m_robotCoral, CoralSubsystem.coralDir.kOUt, .5));
    NamedCommands.registerCommand("Run Intake", new AutonomousCoralSetSpd(m_robotCoral, CoralSubsystem.coralDir.kIn, .2));
    NamedCommands.registerCommand("Elevator Level 0", new AutonomousElevatorSetPos(m_robotElevator, m_coralPivot, elevatorPos.k0, coralPos.k0));
    NamedCommands.registerCommand("Elevator Level 1", new AutonomousElevatorSetPos(m_robotElevator, m_coralPivot, elevatorPos.k1, coralPos.k1));
    NamedCommands.registerCommand("Elevator Level 2", new AutonomousElevatorSetPos(m_robotElevator, m_coralPivot, elevatorPos.k2, coralPos.k3));
    NamedCommands.registerCommand("Elevator Level 3", new AutonomousElevatorSetPos(m_robotElevator, m_coralPivot, elevatorPos.k3, coralPos.k3));
  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    new POVButton(m_mechController, OIConstants.kUpDPad)
      .onTrue(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOut))
      .onFalse(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOff));
    new POVButton(m_mechController, OIConstants.kDownDPad)
      .onTrue(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kIn))
      .onFalse(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOff));

    new POVButton(m_mechController, OIConstants.kLeftDPad)
      .onTrue(new IntakeSetPosCommand(IntakePivotSubsystem.intakePos.k0));
    new POVButton(m_mechController, OIConstants.kRightDPad)
      .onTrue(new IntakeSetPosCommand(IntakePivotSubsystem.intakePos.k1));

    //new JoystickButton(m_mechController, Button.kRightStick.value)
    //  .onTrue(new IntakeSetAngleCommand(m_robotIntake, 0.2))
    //  .onFalse(new IntakeSetAngleCommand(m_robotIntake, 0));
    //new JoystickButton(m_mechController, Button.kLeftStick.value)
    //  .onTrue(new IntakeSetAngleCommand(m_robotIntake, -0.2))
    //  .onFalse(new IntakeSetAngleCommand(m_robotIntake, 0));
    
    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new ElevatorCoralSetPosCommand(ElevatorSubsystem.elevatorPos.k0, CoralPivotSubsystem.coralPos.k0));
    new JoystickButton(m_mechController, Button.kY.value)
      .onTrue(new ElevatorCoralSetPosCommand(ElevatorSubsystem.elevatorPos.k4, CoralPivotSubsystem.coralPos.k4));
    new JoystickButton(m_mechController, Button.kA.value)
      .onTrue(new ElevatorCoralSetPosCommand(ElevatorSubsystem.elevatorPos.k1, CoralPivotSubsystem.coralPos.k1));
    new JoystickButton(m_mechController, Button.kX.value)
      .onTrue(new ElevatorCoralSetPosCommand(ElevatorSubsystem.elevatorPos.k3, CoralPivotSubsystem.coralPos.k3));
    new JoystickButton(m_mechController, Button.kB.value)
      .onTrue(new ElevatorCoralSetPosCommand(ElevatorSubsystem.elevatorPos.k2, CoralPivotSubsystem.coralPos.k2));
    
     new JoystickButton(m_mechController, Button.kLeftBumper.value)
       .onTrue(new CoralSetSpdCommand(m_robotCoral, CoralSubsystem.coralDir.kIn))
       .onFalse(new CoralSetSpdCommand(m_robotCoral, CoralSubsystem.coralDir.kOff));
     new JoystickButton(m_mechController, Button.kRightBumper.value)
       .onTrue(new CoralSetSpdCommand(m_robotCoral, CoralSubsystem.coralDir.kOUt))
       .onFalse(new CoralSetSpdCommand(m_robotCoral, CoralSubsystem.coralDir.kOff));

    /*
    new JoystickButton(m_driverController, Button.kY.value)
      .onTrue(new ClimberSetPosCommand(ClimbSubsystem.climbPos.k0));
    new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new ClimberSetPosCommand(ClimbSubsystem.climbPos.k1));
    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(new ClimberSetPosCommand(ClimbSubsystem.climbPos.k2));
    */

    new POVButton(m_driverController, OIConstants.kDownDPad)
      .onTrue(new IntakeSetPosCommand(IntakePivotSubsystem.intakePos.k2));
    new POVButton(m_driverController, OIConstants.kLeftDPad)
      .whileTrue(new AlignRobotCommand(leftCamera, m_robotDrive, () -> m_robotDrive.getPose2d(), leftCamera.getLatestResult().getBestTarget().getFiducialId()));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .onTrue(new DriveTrainOrientation(m_robotDrive));
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return sc_autonSelect.getSelected();
  }
}
