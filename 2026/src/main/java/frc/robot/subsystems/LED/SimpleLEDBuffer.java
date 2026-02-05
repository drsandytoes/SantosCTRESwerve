package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** 
 * This class represents a portion of an LED string managed by SimpleLEDSubsystem.
 */
public class SimpleLEDBuffer implements SimpleLEDPatternApplier {
    private AddressableLEDBufferView buffer;

    public SimpleLEDBuffer(AddressableLEDBufferView buffer) {
        this.buffer = buffer;
    }

        /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        Command runCommand = Commands.run(() -> pattern.applyTo(buffer));
        return runCommand;
    }

    /**
     * Applies a pattern to the LED strip.
     * 
     * @param pattern the LED pattern to run
     */
    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }
    
}
