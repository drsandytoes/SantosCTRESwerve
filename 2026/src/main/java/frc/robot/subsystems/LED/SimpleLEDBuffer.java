package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * This class represents a portion of an LED string managed by SimpleLEDSubsystem.
 */
public class SimpleLEDBuffer extends SubsystemBase implements SimpleLEDPatternApplier {
    private AddressableLEDBufferView buffer;
    private LEDPattern defaultPattern = LEDPattern.solid(Color.kBlack);

    public SimpleLEDBuffer(AddressableLEDBufferView buffer) {
        this.buffer = buffer;

        this.setDefaultCommand(Commands.run(() -> applyDefaultPattern(), this).ignoringDisable(true));
    }

    private void applyDefaultPattern() {
        defaultPattern.applyTo(buffer);
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
     * Sets the default pattern for the LED strip.
     * 
     * @param pattern the LED pattern to run
     */
    public void applyPattern(LEDPattern pattern) {
        defaultPattern = pattern;
    }
    
}
