package frc.robot.commands.indexing;

//subsystems
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;

//bc it's a command
import edu.wpi.first.wpilibj2.command.Command;


public class UndoAmpIndex extends Command {
    //defining member subsystems
    private final IntakeSubsystem m_intake;
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;
    private final LEDSubsystem m_led;

    //establishing state management
    private enum  reversingStates {
        DOWN,
        UP
    }
    reversingStates currentState = reversingStates.DOWN;
    private boolean indexingSensorFirstPass = false;


    //constructor and command scheduler stuff
    private boolean m_complete = false;

    public UndoAmpIndex(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, LEDSubsystem led) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
        m_led = led;

        addRequirements(m_intake, m_indexer, m_shooter, m_led);
    }

    @Override
    public void initialize() {
        m_complete = false;
        //run everything backwards
        m_intake.intakeRunBackwards();
        m_indexer.runConveyorReverse();
        m_shooter.shooterReverse();
        //initialize states
        currentState = reversingStates.DOWN;
        indexingSensorFirstPass = false;
    }

    @Override
    public void execute() {
        //GOING DOWN
        if (currentState == reversingStates.DOWN) {
            //we want to switch to going up once we know that we've passed the indexing sensor
            //and the indexing sensor is off. That will mean we're below the note.
            if (!m_indexer.getIndexerSensor()) {
                //note once we have passed the indexer sensor
                indexingSensorFirstPass = true;
            }
            if (indexingSensorFirstPass && m_indexer.getIndexerSensor()) {
                //if we have passed the indexing sensor and the indexing sensor is off, that means we've
                //gone far enough, and we now need to go back up to the primary indexed state.
                currentState = reversingStates.UP;
                m_intake.intakeRunForward();
                m_indexer.runConveyorForward();
                m_shooter.shooterOff();
            }
        }

        //GOING UP
        if (currentState == reversingStates.UP) {
            //just keep going up until we hit the indexing sensor, then the note is properly indexed
            if (!m_indexer.getIndexerSensor()) {
                //once we've hit the sensor, stop the motors, and we're done.
                m_intake.intakeStop();
                m_indexer.stopIndexConveyor();
                m_complete = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intakeStop();
        m_indexer.stopIndexConveyor();
        m_shooter.shooterOff();
        m_led.LEDGreen();
    }

    @Override
    public boolean isFinished() {
        return m_complete;
    }
}