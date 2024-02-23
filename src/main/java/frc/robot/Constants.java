// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kMagnitudeSlewRate = 1.8 * kMaxSpeedMetersPerSecond; // meters per second^2
    public static final double kRotationalSlewRate = 2.0 * kMaxAngularSpeed;        // radians per second^2

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 9;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 2;
  }
  public static final class IntakeConstants {
    public static final int KIntakeCanID = 20;
    public static final int kIntakeCurrentLimit = 20; // amps
    public static final double kIntakeMotorSpeed = 0.8;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    //public static final int kDrivingMotorPinionTeeth = 14; //done

    // Inverts the turning encoder.
    public static final boolean kTurningEncoderInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    //public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

    // The L1 MK4 and MK4i modules have a gear ratio of 8.14:1 on the drive wheels.
    public static final double kDrivingMotorReduction = 8.14;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    //TODO: We need to figure out how feedforwards work and how to use them.
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    //inversion of drive motors
    public static final boolean kLeftFrontInverted = false;
    public static final boolean kLeftRearInverted = false;
    public static final boolean kRightFrontInverted = true;
    public static final boolean kRightRearInverted = false;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    //TODO: probably not super important, but we should look into how to properly calculate current limits.
    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 5; // amps
  }

  public static final class HeadingConstants {
    public static final boolean kGyroReversed = false;

    // This is used for making the robot face a certain direction
    public static final double kHeadingP = 0.05;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0.001;
    public static final double kHeadingMinOutput = -0.5;
    public static final double kHeadingMaxOutput = 0.5;
    public static final double kHeadingTolerance = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {

    //How many degrees is your limelight rotated from perfectly vertical
    public static final double kLimelightMountAngle = 0; //NOTE: we should really take into account the rotation of the robot using the pitch of the NavX - Noah

    //Limelight lens height from floor in inches
    public static final double kLimelightLensHeight = 20;

    //Height of reflective tape poles in inches
    public static final double kTopReflectiveTapeHeight = 24;
    public static final double kBottomReflectiveTapeHeight = 24;

    public static final double kTopPoleDesiredDistance = 24;
    public static final double kDistanceTolerance = 2;
    public static final double kMaxForwardSpeed = 0.7;
    public static final double kForwardSpeedPConstant = 0.1;

    public static final double kRotationSpeed = 0.2;
    public static final double kRotationTolerance = 2;

    //Pipeline constants
    public static final int kAprilTagPipeline = 0;
    public static final int kReflectiveTapePipeline = 3;
    public static final int kGamePiecePipeline = 2;

    /* NOTE: the limelight starts with pipeline 0 by default, so we need to make sure we make that pipeline something 
     * that doesn't use the green lights so we don't blind everybody.
     */
    public static final int kDefaultPipeline = kAprilTagPipeline;
  }

  public static final class IndexerConstants {
    //establish all of these once the subsystem is actually wired up yk the deal
    public static final int kIndexerConveyorMotorCanId = 0;
    public static final int kIndexerSensorChannel = 0;
    public static final double kIndexerConveyorStdSpeed = 0.5; //needs to be tested
    public static final boolean kIndexerMotorInverted = false;
  }

  public static final class ClimbConstants {
    //establish all of these once the LEDs are actually established yk the deal
    //climb motor CAN IDs
    public static final int kLeftClimbCanId = 0;
    public static final int kRightClimbCanId = 0;
    //speeds and configurations
    public static final double kClimbMotorSpeed = 0.5;
    public static final boolean kClimbLeftInverted = false;
    public static final boolean kClimbRightInverted = false;
    public static final IdleMode kClimbIdleMode = kBrake;
  }

  public static final class LEDConstants {
    // establish all of these once the LEDs are actually established yk the deal
    public static final int kLEDPort = 1;
    public static final int kLEDLength = 0;

  }
}
