// package frc.robot.commands.indexing;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;


// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;


// public class AutoEjectThroughIntake extends SequentialCommandGroup {
//     private final IndexerSubsystem m_IndexerSubsystem;
//     private final IntakeSubsystem m_IntakeSubsystem;

//     public AutoEjectThroughIntake(IndexerSubsystem indexer, IntakeSubsystem intake) {
//         m_IndexerSubsystem = indexer;
//         m_IntakeSubsystem = intake;

//         addRequirements(m_IndexerSubsystem, m_IntakeSubsystem);

//         addCommands(
//             new EjectThroughIntake(m_IndexerSubsystem, m_IntakeSubsystem),
//             new WaitCommand(2),
//             new InstantCommand(() -> m_IndexerSubsystem.stopIndexConveyor(), m_IndexerSubsystem),
//             new InstantCommand(() -> m_IntakeSubsystem.intakeStop(), m_IntakeSubsystem)
//         );
//     }
// }
