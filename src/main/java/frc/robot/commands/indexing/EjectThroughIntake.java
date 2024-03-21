// package frc.robot.commands.indexing;

// import edu.wpi.first.wpilibj2.command.Command;

// //subsystems
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;

// //bc it's a command
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;


// public class EjectThroughIntake extends Command {
//     private final IndexerSubsystem m_IndexerSubsystem;
//     private final IntakeSubsystem m_IntakeSubsystem;

//     private boolean m_complete = false;

//     public EjectThroughIntake(IndexerSubsystem indexer, IntakeSubsystem intake) {
//         m_IndexerSubsystem = indexer;
//         m_IntakeSubsystem = intake;
//         addRequirements(m_IndexerSubsystem, m_IntakeSubsystem);
//     }
    
//     @Override
//     public void initialize() {
//     }
//     @Override
//     public void execute()
//     {
//         if(m_IntakeSubsystem.getIndexerSensor() && m_IndexerSubsystem.getIndexerSensor()) 
//         {
//             m_IntakeSubsystem.intakeRunBackwards();
//             //m_ampSubsystem.ampToggle();
//         } 
//         else
//         {
//            m_IntakeSubsystem.intakeStop();
//            m_complete = true;
//         }   
//     }

//     @Override
//     public void end(boolean interrupted) {

//         m_IntakeSubsystem.intakeStop();
//     }
// }
