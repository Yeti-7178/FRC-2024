package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.vision.AutoDriveToNote;
import frc.robot.commands.indexing.AutoIndex;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class AutoPickUpNote extends SequentialCommandGroup {
    private final IntakeSubsystem m_intakeSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final IndexerSubsystem m_indexSubsystem;
    private final LEDSubsystem m_ledSubsystem;
    public AutoPickUpNote(IntakeSubsystem intake, VisionSubsystem vision, DriveSubsystem drive, IndexerSubsystem index, LEDSubsystem led) {
        m_intakeSubsystem = intake;
        m_visionSubsystem = vision;
        m_driveSubsystem = drive;
        m_indexSubsystem = index;
        m_ledSubsystem = led;

        addCommands(
            new ParallelDeadlineGroup(
                new AutoIndex(index, intake, null),
                new AutoDriveToNote(m_visionSubsystem, m_driveSubsystem)
            )
        );
    }
}
