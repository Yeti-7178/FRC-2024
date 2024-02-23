//I may want to also include the indexer subsystem to this to make it into a command which both picks up and indexes a note. For now, this assumes an already intook note but including the intake may make things simpler, but less independent

package frc.robot.commands.indexing;

//subsystems
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//bc it's a command
import edu.wpi.first.wpilibj2.command.Command;


public class AutoIndex extends Command {
    
    //my first simple command!!!11!1!1!
    
    
    private final IndexerSubsystem m_indexerSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    private boolean m_complete = false;

    public AutoIndex(IndexerSubsystem indexer, IntakeSubsystem intake) {
        m_indexerSubsystem = indexer;
        m_intakeSubsystem = intake;
        addRequirements(indexer, intake);
    }

    @Override
    public void initialize(){
        if(m_indexerSubsystem.getIndexerSensor()){
            // if both sensors detect then we are full
            m_intakeSubsystem.intakeStop();
            m_indexerSubsystem.stopIndexConveyor();
            SmartDashboard.putString("Index State","FULL");
        }
        else{
            // no sensors so we must be empty
            SmartDashboard.putString("Index State","EMPTY");
        }
        m_complete = false;
    }

    @Override
    public void execute(){

        if(!m_indexerSubsystem.getIndexerSensor()) 
        {
            m_intakeSubsystem.intakeRunForward();
            m_indexerSubsystem.runConveyorForward();
        } 
        else
        {
            m_intakeSubsystem.intakeRunForward();
            m_indexerSubsystem.runConveyorForward();
        }   
    }

    @Override
    public boolean isFinished() {
        return m_complete;
    }
}
