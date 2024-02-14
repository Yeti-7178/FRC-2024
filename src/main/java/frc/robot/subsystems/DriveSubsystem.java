// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create Swerve Modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      ModuleConstants.kLeftFrontInverted,
      true
      );

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      ModuleConstants.kRightFrontInverted,
      false
      );

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      ModuleConstants.kLeftRearInverted,
      true
      );

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      ModuleConstants.kRightRearInverted,
      true
      );

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
  // Note: the NavX takes a second to configure before it can be used. I have seen some teams create the gyro in a separate thread, which might be worth considering.

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private ChassisSpeeds m_prevTarget = new ChassisSpeeds();

  //Field for odometry
  private final Field2d m_field = new Field2d();

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
  );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // NOTE: is this really necessary??
    m_gyro.enableLogging(true);

    // Shuffleboard values
    Shuffleboard.getTab("Swerve").addDouble("Robot Heading", () -> getHeading());
    
    Shuffleboard.getTab("Swerve").addDouble("frontLeft angle", () -> SwerveUtils.angleConstrain(m_frontLeft.getPosition().angle.getDegrees()));
    Shuffleboard.getTab("Swerve").addDouble("frontRight angle", () -> SwerveUtils.angleConstrain(m_frontRight.getPosition().angle.getDegrees()));
    Shuffleboard.getTab("Swerve").addDouble("rearLeft angle", () -> SwerveUtils.angleConstrain(m_rearLeft.getPosition().angle.getDegrees()));
    Shuffleboard.getTab("Swerve").addDouble("rearRight angle", () -> SwerveUtils.angleConstrain(m_rearRight.getPosition().angle.getDegrees()));
    Shuffleboard.getTab("Swerve").add("Field", m_field);
    
    Shuffleboard.getTab("Swerve").addDouble("robot X", () -> getPose().getX());
    Shuffleboard.getTab("Swerve").addDouble("robot Y", () -> getPose().getY());
    
    // Configure the AutoBuilder
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            // Using pythagoras's theorem to find distance from robot center to module
            Math.hypot(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2), // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> false, // Parameter for whether to invert the paths or not (set to false for now)
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    // Update field widget
    m_field.setRobotPose(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose. Note: this also resets the angle of the robot.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0)),
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
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    // Convert the commanded speeds into the correct units for the drivetrain
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    // Get the target chassis speeds relative to the robot
    final ChassisSpeeds targetVel = (fieldRelative ?
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    // Rate limit if applicable
    if(rateLimit) {
      final double
        currentTime = WPIUtilJNI.now() * 1e-6,
        elapsedTime = currentTime - m_prevTime;

      // Side effect: The velocities of targetVel are modified by this function
      SwerveUtils.RateLimitVelocity(
        targetVel, m_prevTarget, elapsedTime,
        DriveConstants.kMagnitudeSlewRate, DriveConstants.kRotationalSlewRate
      );

      // TODO: the previous times and target velocities are only tracked when rate limit is active. 
      // This could potentially be a problem if rate limit is false for an extended period of time and then is suddenly switched on.
      m_prevTime = currentTime;
      m_prevTarget = targetVel;
    }

    // Use the DriveKinematics to calculate the module states
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetVel);

    // Normalizes the wheel speeds (makes sure none of them go above the max speed)
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Set the modules to their desired states
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Sets the robot's heading to a specific angle.
   * 
   * @param angle The angle (in degrees) to set the robot's heading to.
   */
  public void setHeading(double angle) {
    //The angle adjustment may not be cleared by m_gyro.reset(). Double check in testing
    //m_gyro.setAngleAdjustment((angle-getHeading()) * (HeadingConstants.kGyroReversed ? -1.0 : 1.0));
    m_odometry.resetPosition(
      new Rotation2d(Math.toRadians(angle)), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }, 
      getPose()
    );
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //double angle = Rotation2d.fromDegrees(m_gyro.getAngle() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0)).getDegrees();
    //return MathUtil.inputModulus(angle, -180, 180);
    return m_odometry.getPoseMeters().getRotation().getDegrees(); //Check if this is already constrained
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * A function fed into pathplanner to make it work.
   * 
   * @return The robot-relative translational speeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds(){
    // Uses forward kinematics to calculate the robot's speed given the states of the swerve modules.
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
  }

  /**
   * Another function fed into pathplanner to make it work.
   * 
   * @return The robot-relative translational speeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds){
    // This takes the velocities and converts them into precentages (-1 to 1)
    drive((speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond)*0.2, 
          (speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond)*0.2, 
          speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed, 
          false, 
          false);
  }
}
