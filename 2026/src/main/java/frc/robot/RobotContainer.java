// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import dev.doglog.internal.tunable.Tunable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionConfigurationControl;
import frc.robot.subsystems.VisionSimulation;
import frc.robot.subsystems.VisionUpdate;
import frc.robot.subsystems.LED.SimpleLEDSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller joystick = new CommandPS4Controller(0);

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionUpdate visionUpdater;
    private final VisionConfigurationControl visionControl;
    private final SimpleLEDSubsystem ledSubsystem;

    // Vision simulation
    private VisionSimulation visionSim;
    private DoubleSubscriber visionSimDistance = DogLog.tunable("VisionSim/dist", 1.0, (double val) -> {
        visionSim.setSimulatedDistance(val);
    });
    private DoubleSubscriber visionSimRotation = DogLog.tunable("VisionSim/rotationDeg", 180.0, (double val) -> {
        visionSim.setSimulatedRotation(Rotation2d.fromDegrees(val));
    });
    private IntegerSubscriber visionSimTagID = DogLog.tunable("VisionSim/tag", 12, (long val) -> {
        visionSim.setTargetTag((int)val);
    });

    private final NotifyingAutoChooser autoChooser;
    private final AutoFactory autoFactory;

    public RobotContainer() {
        visionSim = new VisionSimulation();

        // Get initial sim target from the tunables
        visionSim.setTargetTag((int)visionSimTagID.get());
        visionSim.setSimulatedDistance(visionSimDistance.get()); 
        visionSim.setSimulatedRotation(Rotation2d.fromDegrees(visionSimRotation.get())); 

        ledSubsystem = new SimpleLEDSubsystem();
        visionUpdater = new VisionUpdate(drivetrain, Constants.Vision.limelightName, visionSim);

        // Purposefully only control part of the string for demo purposes. This would be
        // useful if
        // we used different segments of lights for different purposes.
        var ledSubString = ledSubsystem.getBuffer(16, 31);
        visionControl = new VisionConfigurationControl(visionUpdater, drivetrain, ledSubString);
        configureBindings();

        // Create the auto factory
        autoFactory = new AutoFactory(
            () -> drivetrain.getState().Pose, // A function that returns the current robot pose
            visionControl::resetPoseForChoreoStart, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
    );


        // Create the auto chooser
        autoChooser = new NotifyingAutoChooser();

        // Add options to the chooser
        autoChooser.addRoutine("Example HTW Routine", this::htw);
        autoChooser.addRoutine("Example OTCTW Routine", this::otctw);

        autoChooser.onChange(this::selectedAutoChanged);

        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Choices", autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Reset the vision controller. Forcably if share button is also pressed
        joystick.options().onTrue(Commands.run(() -> visionControl.reset(joystick.share().getAsBoolean())));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.circle().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.PS().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.PS().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

    // Callbacks from Robot.java for disabled mode entry/exit
    public void disabledInit() {
        visionControl.robotEnabled(false);
    }

    public void disabledExit() {
        visionControl.robotEnabled(true);
    }

    public void driverStationConnected() {
        // Re-evalute the starting pose because it might now be flipped
        selectedAutoChanged(autoChooser.lastKnownSelection());
    }

    // Choreo routines
    private AutoRoutine htw() {
        AutoRoutine routine = autoFactory.newRoutine("HTW");
        // Load the routine's trajectories
        // Optional<Trajectory<SwerveSample>> trajectory =
        // Choreo.loadTrajectory("test");
        AutoTrajectory hubtowershoot = routine.trajectory("H_TW");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        hubtowershoot.resetOdometry(),
                        hubtowershoot.cmd()

                ));

        return routine;
    }

    private AutoRoutine otctw() {
        AutoRoutine routine = autoFactory.newRoutine("OTCTW");
        // Load the routine's trajectories
        // Optional<Trajectory<SwerveSample>> trajectory =
        // Choreo.loadTrajectory("test");
        AutoTrajectory fuelshootclimb = routine.trajectory("OT_C_TW");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        fuelshootclimb.resetOdometry(),
                        fuelshootclimb.cmd()

                ));

        return routine;
    }

    // Callback that happens whenever auto changes
    private void selectedAutoChanged(String newAutoName) {
        System.out.println("New auto selected: " + newAutoName);

        // Tell the VisionControl system to start over with the new pose
        Optional<Trajectory<SwerveSample>> trajectory = Optional.empty();
        switch (newAutoName) {
            case "Example OTCTW Routine": // Should use shared string constants to avoid mismatches here!
                trajectory = Choreo.loadTrajectory("OT_C_TW");
                break;
            case "Example HTW Routine":
                trajectory = Choreo.loadTrajectory("H_TW");
                break;
            default:
                System.out.println("WARNING: unknown auto name!");
                break;
        }
        if (trajectory.isPresent()) {
            var alliance = DriverStation.getAlliance();
            boolean isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;

            Optional<SwerveSample> startingSample = trajectory.get().getInitialSample(isRedAlliance);
            if (startingSample.isPresent()) {
                Pose2d newStartingPose = startingSample.get().getPose();
                visionControl.setNewStartingPose(newStartingPose);
                DogLog.log("RobotContainer/StartingPose", newStartingPose);
            }
        }
    }

    public void disableVisionSim() {
        visionSim.setEnabled(false);
    }

}
