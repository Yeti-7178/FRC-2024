package frc.robot.commands.vision;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.HeadingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class AutoDriveToNote  extends Command {
    private final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    private double robotRelativeNoteAngle;
    private double robotRelativeNoteDistance;
    private double angleToDriveTo;
    private double rotation;

    //for debugging
    private int executeIncremental = 0;

    private boolean m_complete = false;

    private final PIDController pidController = new PIDController(0.025, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);


    public AutoDriveToNote(VisionSubsystem vision, DriveSubsystem drive){
        m_visionSubsystem = vision;
        m_driveSubsystem = drive;

        addRequirements(vision, drive);
    }
    
    @Override
    public void initialize() {
        m_complete = false;
        if(m_visionSubsystem.getNoteTClass() == "note")
        {
            robotRelativeNoteAngle = -m_visionSubsystem.getNoteTX()/* * 29.8 / 320 */;
            
            angleToDriveTo = m_driveSubsystem.getHeading() + robotRelativeNoteAngle;
            
            pidController.setSetpoint(angleToDriveTo);
        }
    }


    @Override
    public void execute() {
        //should be the same as what's in init, just setting the desired rotation output
        robotRelativeNoteAngle = -m_visionSubsystem.getNoteTX()/* * 59.6 / 320*/;
        angleToDriveTo = m_driveSubsystem.getHeading() + robotRelativeNoteAngle;
        pidController.setSetpoint(angleToDriveTo);
        rotation = pidController.calculate(m_driveSubsystem.getHeading());
        // if (Math.abs(robotRelativeNoteAngle) < 5) {
            m_driveSubsystem.drive(-0.4, 0, -rotation, false, false);
        // } else {
        //     m_driveSubsystem.drive(0, 0, -rotation, false, false);
        //     //m_complete = true;
        // }
        // Shuffleboard.getTab("Auto pick-up testing").add("robotRelativeNoteAngle", robotRelativeNoteAngle);
        // Shuffleboard.getTab("Auto pick-up testing").add("angleToDriveTo", angleToDriveTo);
        SmartDashboard.putNumber("robotRelativeNoteAngle", robotRelativeNoteAngle);
        SmartDashboard.putNumber("angleToDriveTo", angleToDriveTo);
        SmartDashboard.putNumber("rotation value", rotation);

        //debugging, making sure execute is running periodically
        executeIncremental++;
        SmartDashboard.putNumber("executeIncremental", executeIncremental);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, true, true);
        m_complete = true;
        SmartDashboard.putBoolean("AutoDrive interrupted", interrupted);
        
    }

    @Override
    public boolean isFinished() {
        return m_complete;
    }
    
}
