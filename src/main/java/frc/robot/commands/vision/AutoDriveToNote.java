package frc.robot.commands.vision;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.HeadingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDriveToNote  extends Command {
    private final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    private double robotRelativeNoteAngle;
    private double robotRelativeNoteDistance;
    private double angleToDriveTo;
    private double roatation;

    private final PIDController pidController = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);


    public AutoDriveToNote(VisionSubsystem vision, DriveSubsystem drive){
        m_visionSubsystem = vision;
        m_driveSubsystem = drive;

        addRequirements(vision, drive);
    }
    
    @Override
    public void initialize() {
        robotRelativeNoteAngle = -m_visionSubsystem.getNoteTX() * 59.6 / 320;
        angleToDriveTo = m_driveSubsystem.getHeading() + robotRelativeNoteAngle;
        pidController.setSetpoint(angleToDriveTo);
    }

    @Override
    public void execute() {
        robotRelativeNoteAngle = -m_visionSubsystem.getNoteTX() * 59.6 / 320;
        roatation = pidController.calculate(m_driveSubsystem.getHeading());
        if (Math.abs(robotRelativeNoteAngle) < 5) {
            m_driveSubsystem.drive(0.4, 0, roatation, false, true);
        } else {
            m_driveSubsystem.drive(0, 0, roatation, false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, true, true);
    }
}
