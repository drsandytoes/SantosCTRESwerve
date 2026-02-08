// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private Alliance currentAllianceColor = Alliance.Blue;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        m_robotContainer.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        // Watch for alliance color changes while disabled. Probalby don't need this for real robot code, but
        // it's useful for testing.
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Alliance newAlliance = alliance.get();
            if (newAlliance != currentAllianceColor) {
                currentAllianceColor = newAlliance;

                // Simulate a DS connection event so we re-evaluate our state.
                m_robotContainer.driverStationConnected();
            }
        }
    }

    @Override
    public void disabledExit() {
        m_robotContainer.disabledExit();
    }

    @Override
    public void autonomousInit() {
        // Don't do this. Choreo uses a RobotModeTrigger instead.
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // if (m_autonomousCommand != null) {
        //     CommandScheduler.getInstance().schedule(m_autonomousCommand);
        // }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void driverStationConnected() {
        // We need to re-evalute our starting pose. Forward it on to RobotContainer.
        // Unfortunately, this is only called once, so if you disconnect, change the alliance color
        // and reconnect, we won't be called again.
        m_robotContainer.driverStationConnected();
    }
}
