package frc.robot.commands.vision;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPickNote  extends Command{

    private final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final IntakeSubsystem m_intakeSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    public AutoPickNote(VisionSubsystem vision, DriveSubsystem drive, IntakeSubsystem intake){
        m_visionSubsystem = vision;
        m_driveSubsystem = drive;
        m_intakeSubsystem = intake;

        addRequirements(vision, drive, intake);
    }
    
}
