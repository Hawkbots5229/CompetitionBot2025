package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;

//import ;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.DriveConstants;
import frc.robot.library.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;



public class DrivetrainSubsystem extends SubsystemBase{
    private final SwerveModule m_frontLeft =
      new SwerveModule(DriveConstants.SDFrontLeft);

  private final SwerveModule m_rearLeft =
      new SwerveModule(DriveConstants.SDRearLeft);

  private final SwerveModule m_frontRight =
      new SwerveModule(DriveConstants.SDFrontRight);

  private final SwerveModule m_rearRight =
      new SwerveModule(DriveConstants.SDRearRight);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final PhotonCamera camera = new PhotonCamera("Arducam1");
  private final PhotonPipelineResult result = new PhotonPipelineResult();
  private final PhotonTrackedTarget target = new PhotonTrackedTarget();
  private final AprilTagFieldLayout aprilTag = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  // Sets if robot drive is field relative Change it to True
  public boolean isFieldRelative = false; 

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    resetEncoders();
    zeroHeading();

    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e){
      System.out.println("Error Loading Config: " + e.getMessage());
      throw new RuntimeException("Unable to initialize RobotConfig.");
    }

    AutoBuilder.configure(
      () -> this.getPose(),
      pose -> this.resetOdometry(getPose()),
      () -> new ChassisSpeeds(getVelocity(), 0, 0),
      (speeds, feedforwards) -> driveRobotRelativeSpeeds(speeds, true),
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 5.0, 5.0)
      ),
      config,
      () ->false,
      this
    );
  
  }

  public List <PhotonPipelineResult> getLastResult() {
    return camera.getAllUnreadResults();
  };

  public boolean checkTargets() {
    return result.hasTargets();
  }

  /** Updates the robot odometry.
   * 
   * @return Void
   * @param None
   * @implNote edu.wpi.first.math.kinematics.SwerveDriveOdometry.update()
   * @implNote edu.wpi.first.wpilibj.interfaces.Gyro.getRotation2d()
   * @implNote frc.robot.library.SwerveModule.getPosition()
   * 
   */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The current pose of the robot (meters)
   * @param None
   * @implNote edu.wpi.first.math.kinematics.SwerveDriveOdometry.getPoseMeters()
   * 
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose3d getPose3d() {
    if (aprilTag.getTagPose(target.getFiducialId()).isPresent()){
      return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTag.getTagPose(target.getFiducialId()).get(), new Transform3d());
    } else {
      return null;
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @return Void
   * @param pose The pose to which to set the odometry.
   * @implNote edu.wpi.first.math.kinematics.SwerveDriveOdometry.resetPosition()
   * @implNote edu.wpi.first.wpilibj.interfaces.Gyro.getRotation2d()
   * @implNote frc.robot.library.SwerveModule.getPosition()
   * 
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @return Void
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param optimize Whether to optimize wheel rotations
   * 
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean isFieldRelative, boolean optimize) {
    //this.isFieldRelative = isFieldRelative;
    
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0], optimize, false, false);
    m_frontRight.setDesiredState(swerveModuleStates[1], optimize, false, false);
    m_rearLeft.setDesiredState(swerveModuleStates[2], optimize, false, false);
    m_rearRight.setDesiredState(swerveModuleStates[3], optimize, false, false);
  }

  public void driveRobotRelativeSpeeds(ChassisSpeeds speeds, boolean optimize) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0], optimize, false, false);
    m_frontRight.setDesiredState(swerveModuleStates[1], optimize, false, false);
    m_rearLeft.setDesiredState(swerveModuleStates[2], optimize, false, false);
    m_rearRight.setDesiredState(swerveModuleStates[3], optimize, false, false);
  }

  /** Resets the swerve drive and turning motor encoders to a position of 0. 
   * 
   * @return Void
   * @param None
   * @implNote frc.robot.library.SwerveModule.resetEncoders()
   * 
  */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Resets the swerve drive motor encoders to a position of 0. 
   * 
   * @return Void
   * @param None
   * @implNote frc.robot.library.SwerveModule.restDriveEncoders()
   * 
  */
  public void restDriveEncoders() {
    m_frontLeft.resetDriveEncoders();
    m_rearLeft.resetDriveEncoders();
    m_frontRight.resetDriveEncoders();
    m_rearRight.resetDriveEncoders();
  }

  /** Zeroes the heading of the robot. 
   * 
   * @return Void
   * @param None
   * @implNote com.kauailabs.navx.frc.AHRS.reset()
   */
  public void zeroHeading() {
    m_gyro.reset();
    System.out.println("Zero Heading");
  }

  /** Returns the heading of the robot.
   * 
   * @return The robot's heading from -180 to 180 (degrees)
   * @param None
   * @implNote edu.wpi.first.wpilibj.interfaces.Gyro.getRotation2d()
   * @implNote edu.wpi.first.math.geometry.Rotation2d.getDegrees()
   * 
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /** Returns the turn rate of the robot. Direction of rate can be changed with DriveConstants
   * 
   * @return The turn rate of the robot(Deg/Sec)
   * @param None
   * @implNote com.kauailabs.navx.frc.AHRS.getRate()
   * @implNote DriveConstants.kGyroReversed
   * 
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Gets distance the robot has driven in meters. This is the average of the distance each motor has driven.
   * 
   * @return Distance (meters)
   * @param None
   * @implNote frc.robot.library.SwerveModule.getDriveDistanceMeters()
   * 
   */
  public double getDriveDistanceMeters(){
    double dis = (m_frontLeft.getDriveDistanceMeters() + m_rearLeft.getDriveDistanceMeters() + m_frontRight.getDriveDistanceMeters() + m_rearRight.getDriveDistanceMeters())/4.0;
    return dis;
  }

  /** Gets distance the robot has driven in inches. This is the average of the distance each motor has driven.
   * 
   * @return Distance (meters)
   * @param None
   * @implNote getDriveDistanceMeters()
   * @implNote DriveConstants.MetersPerInch
   * 
   */
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() / DriveConstants.MetersPerInch;
  }

  /** Gets the drive speed of the wheels in meters/second.  This is the average of the velocity of each motor
   * 
   * @return Velocity (meters/second)
   * @param None
   * @implNote getDriveVelocity()
   * 
   */
  public double getVelocity() {
    return 
    (m_frontLeft.getDriveVelocity()
    + m_frontRight.getDriveVelocity()
    + m_rearLeft.getDriveVelocity()
    + m_rearRight.getDriveVelocity())/4;
  }

  /** Returns the angle of the robot. Direction of rate can be changed with DriveConstants
   * 
   * @return Returns the total accumulated yaw angle (Z Axis, in degrees)
   * @param None
   * @implNote com.kauailabs.navx.frc.AHRS.getAngle()
   * @implNote DriveConstants.kGyroReversed
   * 
   */
  public double getRobotAngle() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Returns the angle of the robot in 360 degrees. Direction of rate can be changed with DriveConstants
   * 
   * @return Returns the yaw angle (Z Axis, in degrees)
   * @param None
   * @implNote com.kauailabs.navx.frc.AHRS.getAngle()
   * @implNote DriveConstants.kGyroReversed
   * 
   */
  public double getRobotAngle360() {
    return (m_gyro.getAngle()%360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0) ;
  }

  /** Returns the pitch of the robot.
   * 
   * @return Returns the current pitch value (in degrees, from -180 to 180)
   * @param None
   * @implNote com.kauailabs.navx.frc.AHRS.getPitch()
   * 
   */
  public double getRobotPitch() {

    return m_gyro.getPitch(); 
  }

  /** Returns the 2d rotaton of the robot.
   * 
   * @return 2D rotation object
   * @param None
   * @implNote getRobotAngle()
   * @implNote edu.wpi.first.math.geometry.Rotation2d.fromDegrees()
   * 
   */
  public Rotation2d getRobotRotation2D(){
    double angle = getRobotAngle360();
    return Rotation2d.fromDegrees(angle);
  }

  /** Used to change whether the robot is field oriented.
   * 
   * @return Void
   * @param frm Indicates if the robot should be field oriented
   * 
   */
  public void setFieldRelative(boolean frm){
    this.isFieldRelative = frm;
  }

  /** Stops all the drive motors.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.CANSparkMax.stopMotor()
   * 
   */
  public void stopMotors(){
    m_frontLeft.stopMotors();
    m_rearLeft.stopMotors();
    m_frontRight.stopMotors();
    m_rearRight.stopMotors();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    
    //SmartDashboard.putNumber("RobotPitch", getRobotPitch());
    //SmartDashboard.putNumber("Robot Angle", getRobotRotation2D().getDegrees());
    SmartDashboard.putNumber("Robot Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Robot Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("Robot Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Robot Rotation", getRobotRotation2D().getDegrees());
    SmartDashboard.putNumber("Robot Rotation Raw", m_gyro.getRotation2d().getDegrees());
    m_frontLeft.sendData();
    m_frontRight.sendData();
    m_rearLeft.sendData();
    m_rearRight.sendData();
    
  }
}
