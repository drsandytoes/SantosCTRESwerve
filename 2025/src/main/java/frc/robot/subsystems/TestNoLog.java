package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestNoLog extends SubsystemBase {
    @Logged private long loggedCounter = 0;

    public TestNoLog() {
        loggedCounter = 0;
    }

    public void periodic() {
        loggedCounter++;
    }
    
}
