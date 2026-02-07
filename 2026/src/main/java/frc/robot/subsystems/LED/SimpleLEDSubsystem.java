package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimpleLEDSubsystem extends SubsystemBase implements SimpleLEDPatternApplier {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private LEDPattern defaultPattern = LEDPattern.solid(Color.kBlack);

    public SimpleLEDSubsystem() {
        m_led = new AddressableLED(Constants.LED.port);
        m_buffer = new AddressableLEDBuffer(Constants.LED.stringLength);
        m_led.setLength(Constants.LED.stringLength);
        m_led.start();

        // If we don't use sub-buffers, we should apply a default command here that applies
        // the defaultPattern. But if we do that when using sub-buffers, they'll conflict.
    }

    @Override
    public void periodic() {
        // Periodically send the latest LED color data to the LED strip for it to
        // display
        m_led.setData(m_buffer);
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        Command runCommand = run(() -> pattern.applyTo(m_buffer));
        runCommand.addRequirements(this);
        return runCommand;
    }

    /**
     * Sets the default pattern for the LED strip.
     * 
     * @param pattern the LED pattern to run
     */
    public void applyPattern(LEDPattern pattern) {
        defaultPattern = pattern;
    }

    public SimpleLEDBuffer getBuffer(int start, int end) {
        return new SimpleLEDBuffer(m_buffer.createView(start, end));
    }
}