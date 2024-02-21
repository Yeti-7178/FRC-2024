//I may want to also include the indexer subsystem to this to make it into a command which both picks up and indexes a note. For now, this assumes an already intook note but including the intake may make things simpler, but less independent

package frc.robot.commands.drive;

//subsystems
import frc.robot.subsystems.IndexerSubsystem;
//bc it's a command
import edu.wpi.first.wpilibj2.command.Command;


public class IndexANote extends Command {
    //my first simple command!!!11!1!1!
    private final IndexerSubsystem m_indexerSubsystem;

    public IndexANote(IndexerSubsystem indexer) {
        m_indexerSubsystem = indexer;
        addRequirements(indexer);
    }

    @Override //I have no idea what "@Override" does. I need to figure that out sometime
    public void initialize() {
        m_indexerSubsystem.runConveyorForward();
    }

    @Override
    public void end() {
        m_indexerSubsystem.stopIndexConveyor();
    }

    @Override
    public void isFinished() {
        return m_indexerSubsystem.getIndexerSensor();
    }
}
