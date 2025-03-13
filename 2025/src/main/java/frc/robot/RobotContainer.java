// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DefaultDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestNoLog;
import frc.robot.subsystems.Vision;
import frc.robot.utils.MathUtils;

@Logged
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    @NotLogged private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    @NotLogged private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @NotLogged private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    @NotLogged private final Telemetry logger = new Telemetry(MaxSpeed);

    @NotLogged private final CommandPS4Controller joystick = new CommandPS4Controller(0);

    @NotLogged public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Vision vision = new Vision(drivetrain, RobotConstants.Vision.limelightName);
    final TestNoLog test = new TestNoLog();
    public DefaultDrive defaultDriveCommand;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Split out as a separate ivar so it gets logged?
        defaultDriveCommand = new DefaultDrive(() -> -joystick.getLeftY(), () -> -joystick.getLeftX(),
        () -> -joystick.getRightX(), drivetrain);
        drivetrain.setDefaultCommand(defaultDriveCommand);

        joystick.share().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.options().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on touchpad press. CTRE just calls seedFieldRelative() by default, but that
        // just records an offset without adjusting the pose, which really makes a heading controller unhappy. We really
        // want to reset the pose here to be facing forward.
        // CTRE: joystick.touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        joystick.touchpad().onTrue(drivetrain.runOnce(() -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                resetPoseForward(alliance.get());
            } else {
                resetPoseForward(Alliance.Blue);
            }
        }));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void resetPoseForward(Alliance alliance) {
        if (alliance == Alliance.Red) { 
            // Start at red origin, but facing blue alliance wall
            DataLogManager.log("RESETTING FORWARD: RED ALLIANCE");
            drivetrain.resetRotation(MathUtils.rotation.kHalf);
        } else {
            // Start at blue origina
            DataLogManager.log("RESETTING FORWARD: BLUE ALLIANCE");
            drivetrain.resetRotation(MathUtils.rotation.kZero);
        }
    }

}
