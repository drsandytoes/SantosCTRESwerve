// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.DefaultDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MathUtils;
import frc.robot.util.VisionUtils.TimestampedVisionUpdate;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionIO;
import frc.robot.vision.VisionIOLimelight;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandPS4Controller joystick = new CommandPS4Controller(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final Vision limelight;
    private final VisionSystemSim visionSim;


    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(RobotConstants.Drivetrain.MaxSpeed);

    private LoggedDashboardChooser<Command> autoChooser;

    private void configureBindings() {
        // Joystick X is left/right, and Y is up/down, but field X is forward, and Y is
        // sideways. Also, axes are all inverted.
        // We invert the inputs and map them to the corret axes so everything else can
        // deal with FRC coordinates.
        drivetrain.setDefaultCommand(new DefaultDrive(() -> -joystick.getLeftY(), () -> -joystick.getLeftX(),
                () -> -joystick.getRightX(), drivetrain));

        joystick.share().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.options().whileTrue(drivetrain
                .applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

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
        
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(MathUtils.translation.kZero, MathUtils.rotation.kQuarter));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = new LoggedDashboardChooser<Command>("Auto Path", AutoBuilder.buildAutoChooser("NearNotes"));

        switch (RobotConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                limelight = new Vision(this::applyVisionUpdates, new VisionIOLimelight(RobotConstants.Vision.limelightName, 
                    () -> drivetrain.getState().Pose.getRotation(),
                    () -> Units.DegreesPerSecond.of(-drivetrain.getPigeon2().getRate()).in(Units.RotationsPerSecond)));
                limelight.useVision(RobotConstants.Vision.enabled);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                visionSim = new VisionSystemSim("main");
                

                break;

            default:
                // Replayed robot, disable IO implementations
                limelight = new Vision(this::applyVisionUpdates, new VisionIO() {});
                limelight.useVision(RobotConstants.Vision.enabled);
                break;
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void setStartingPoseForAlliance(Alliance alliance) {
        DataLogManager.log("SEEDING INITIAL POSITION");
        if (alliance == Alliance.Red) { 
            // Start at red origin, but facing blue alliance wall
            DataLogManager.log("INITIAL POSITION: RED ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(MathUtils.translation.kZero, MathUtils.rotation.kHalf));
        } else {
            // Start at blue origina
            DataLogManager.log("INITIAL POSITION: BLUE ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(MathUtils.translation.kZero, MathUtils.rotation.kZero));
        }
    }

    public void resetPoseForward(Alliance alliance) {
        Pose2d currentPose = drivetrain.getState().Pose;
        if (alliance == Alliance.Red) { 
            // Start at red origin, but facing blue alliance wall
            DataLogManager.log("RESETTING FORWARD: RED ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(currentPose.getX(), currentPose.getY(), MathUtils.rotation.kHalf));
        } else {
            // Start at blue origina
            DataLogManager.log("RESETTING FORWARD: BLUE ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(currentPose.getX(), currentPose.getY(), MathUtils.rotation.kZero));
        }
    }

    /**
     * Method to consume vetted TimestampedVisionUpdates and apply them to odometry
     * 
     * @param updates Vetted updates to apply to the drivetrain odometry
     */
    private void applyVisionUpdates(TimestampedVisionUpdate update) {
        drivetrain.addVisionMeasurement(update.pose(), update.timestamp(), update.stdDevs());
    }

    private void setupVisionSim() {
        VisionSystemSim visionSim = new VisionSystemSim("main");

    }
}
