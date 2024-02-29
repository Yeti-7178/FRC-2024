package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AmpSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class AutoAlignAndShoot extends ParallelCommandGroup {
    private final IndexerSubsystem m_indexerSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final AmpSubsystem m_ampSubsystem;
    private boolean hasPassedAmpSensor = false;

    private boolean m_complete = false;

    public AutoAlignAndShoot(IndexerSubsystem indexer, ShooterSubsystem shooter, AmpSubsystem amp) {
        m_indexerSubsystem = indexer;
        m_shooterSubsystem = shooter;
        m_ampSubsystem = amp;
        addRequirements(indexer, shooter, amp);


        addCommands(
                new SequentialCommandGroup
                (
                    new ParallelCommandGroup
                    (
                        new InstantCommand(m_shooterSubsystem::shooterOn),
                        new WaitCommand(2)
                    ),
                   new InstantCommand(m_indexerSubsystem::runConveyorForward),
                   new WaitCommand(2),
                   new InstantCommand(m_shooterSubsystem::setSlowSpeed)
                )
        );
    }


}
