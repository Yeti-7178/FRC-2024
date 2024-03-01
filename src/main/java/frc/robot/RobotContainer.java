// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.TemplateAuton;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.commands.vision.AutoAlignAutoAim;
import frc.robot.commands.vision.AutoAlignCircle;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.commands.vision.UpdateOdometry;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.indexing.AutoIndex;
import frc.robot.commands.indexing.AutoIndexAmp;
import frc.robot.commands.shooter.AutoAlignAndShoot;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//vision
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Map;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final AmpSubsystem m_ampSubsystem = new AmpSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  Thread m_visionThread;
  // uncomment this once LEDs are addded
  // private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  // The driver's controller
  // final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  final XboxController m_opperatorController = new XboxController(OIConstants.kOpperatorControllerPort);

  SendableChooser<Command> m_autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_ampSubsystem.enableCompressor();
    m_shooterSubsystem.setSlowSpeed();
    // Configure limelight default pipeline
    m_visionSubsystem.setDefaultCommand(new DefaultLimelightPipeline(m_visionSubsystem));

    //Vision Activation
    m_visionThread =
        new Thread(
           () -> {
             // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
             // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> m_robotDrive.drive(
            MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true, true),
        m_robotDrive));
        // new RunCommand(
        //     () -> m_robotDrive.drive(
        //         -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
        //         true, true),
        //     m_robotDrive));
    
    // "registerCommand" lets pathplanner identify our commands
    //Here's the autoalign as an example:
    NamedCommands.registerCommand("Auto Align", new AutoAlignAutoAim(m_visionSubsystem, m_robotDrive));
    
    NamedCommands.registerCommand("AutoIndex", new AutoIndex(m_indexerSubsystem, m_intakeSubsystem, m_shooterSubsystem));

    //Adding options to the sendable chooser
    m_autonChooser.setDefaultOption("Template Auton", new TemplateAuton(m_robotDrive));
    m_autonChooser.addOption("Path Planner", new PathPlannerAuto("test"));

    // Put chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_autonChooser).withSize(2, 1)
      .withProperties(Map.of("Title", "Auton Command"));

    // DEBUG: shuffleboard widget for resetting pose. For now I'm using a default pose of 0, 0 and a rotation of 0
    Shuffleboard.getTab("Swerve").add("reset pose", new InstantCommand(this::resetPose)).withSize(2, 1);
    Shuffleboard.getTab("Swerve").add("deploy chop", new InstantCommand((m_ampSubsystem::ampToggle)));
    Shuffleboard.getTab("Sensor").add("Sensor", m_ampSubsystem.getAmpIndex());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Right bumper: puts drive into x mode
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    //Left bumper: sets gyro to 0 degrees
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    
    //Y button: auto aim (high pole) (i set it to be on a button press, not held)
    // new JoystickButton(m_driverController, Button.kY.value)
    //     .toggleOnTrue(
    //         new AutoAlignCircle(m_visionSubsystem, m_robotDrive)
    //     );
    
    //A button: makes robot face 0 degrees
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(
            new RobotGotoAngle(
              m_robotDrive,
              0,
              () -> m_driverController.getLeftY(),
              () -> m_driverController.getLeftX()
            )
        );

    
    //B button: sets gyro to 90 degrees
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.setHeading(90),
            m_robotDrive));

    new JoystickButton(m_opperatorController, Button.kB.value)
        .onTrue(new AutoIndex(m_indexerSubsystem, m_intakeSubsystem, m_shooterSubsystem));
    new JoystickButton(m_opperatorController, Button.kA.value)
        .onTrue(new AutoAlignAndShoot(m_indexerSubsystem, m_shooterSubsystem, m_ampSubsystem));
    new JoystickButton(m_opperatorController, Button.kY.value)
        .onTrue(new AutoIndexAmp(m_indexerSubsystem, m_shooterSubsystem, m_ampSubsystem));
    new JoystickButton(m_opperatorController, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_ampSubsystem.ampToggle()))
        .onFalse(new InstantCommand(() -> m_ampSubsystem.ampToggle()));
    new JoystickButton(m_opperatorController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_climbSubsystem.climbUp()))
        .onFalse(new InstantCommand(() -> m_climbSubsystem.climbStop()));
    new JoystickButton(m_opperatorController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_climbSubsystem.climbDown()))
        .onFalse(new InstantCommand(() -> m_climbSubsystem.climbStop()));


  }

  public void resetPose(){
    m_robotDrive.resetOdometry(
        new Pose2d(0, 0, new Rotation2d(0))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }
}
