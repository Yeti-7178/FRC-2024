//I may want to also include the indexer subsystem to this to make it into a command which both picks up and indexes a note. For now, this assumes an already intook note but including the intake may make things simpler, but less independent

package frc.robot.commands.indexing;

//subsystems
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AmpSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//bc it's a command
import edu.wpi.first.wpilibj2.command.Command;


public class AutoIndexAmp extends Command {
    
    private final IndexerSubsystem m_indexerSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final AmpSubsystem m_ampSubsystem;

    private boolean m_complete = false;

    public AutoIndexAmp(IndexerSubsystem indexer, ShooterSubsystem shooter, AmpSubsystem amp) {
        m_indexerSubsystem = indexer;
        m_shooterSubsystem = shooter;
        m_ampSubsystem = amp;
        addRequirements(indexer, shooter, amp);
    }

    @Override
    public void initialize(){
        if(m_ampSubsystem.getAmpIndex()){
            // if both sensors detect then we are full
            m_indexerSubsystem.stopIndexConveyor();
            m_shooterSubsystem.shooterOff();
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

        if(m_ampSubsystem.getAmpIndex()) 
        {
            m_indexerSubsystem.runConveyorForward();
            m_shooterSubsystem.setSpeed(.1);
            //m_ampSubsystem.ampToggle();
        } 
        else
        {
            m_shooterSubsystem.shooterOff();
            m_indexerSubsystem.stopIndexConveyor();
            m_complete = true;
        }   
    }

    @Override
    public void end (boolean interrupted) {
        //m_ampSubsystem.ampToggle();
    }
    @Override
    public boolean isFinished() {
        return m_complete;
    }
}   