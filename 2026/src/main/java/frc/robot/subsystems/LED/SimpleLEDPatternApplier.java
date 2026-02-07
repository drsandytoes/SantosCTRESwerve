package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public interface SimpleLEDPatternApplier {
  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern);

  /**
   * Sets the default pattern to apply.
   * 
   * @param pattern the LED pattern to apply
   */
  public void applyPattern(LEDPattern pattern);
}
