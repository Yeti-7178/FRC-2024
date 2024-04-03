package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// commands/wpi
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//subsystems
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ResetScoringSystems extends SequentialCommandGroup {
    private final IntakeSubsystem m_intake;
    private final IndexerSubsystem m_indexer;
    private final AmpSubsystem m_amp;
    private final ShooterSubsystem m_shooter;
    private final LEDSubsystem m_led;

    public ResetScoringSystems(IntakeSubsystem intake, IndexerSubsystem index, AmpSubsystem amp, ShooterSubsystem shooter, LEDSubsystem led) {
        m_intake = intake;
        m_indexer = index;
        m_amp = amp;
        m_shooter = shooter;
        m_led = led;

        addCommands(
            new InstantCommand(() -> {
                m_intake.intakeStop();
                m_indexer.stopIndexConveyor();
                m_shooter.shooterOff();
                //set the LEDS
                //yellow if indexed at amp (the sensor seems to be inverted)
                if (!m_amp.getAmpIndex()) m_led.LEDYellow();
                //green if at standard index state (I'm gonna bet this sensor is also inverted)
                else if (!m_indexer.getIndexerSensor()) m_led.LEDGreen();
                //Yeti when nothing's active or in the bot
                else m_led.LEDYeti();
            })
        );
    }
}
