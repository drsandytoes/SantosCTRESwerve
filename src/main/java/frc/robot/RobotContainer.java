// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.DefaultDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MathUtils;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightIO;
import frc.robot.vision.LimelightIOReal;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandPS4Controller joystick = new CommandPS4Controller(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final Limelight limelight;

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
            drivetrain.seedFieldRelative(new Pose2d(MathUtils.translation.zero, MathUtils.rotation.quarter));
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
                limelight = new Limelight(new LimelightIOReal(), drivetrain);
                limelight.useLimelight(RobotConstants.Vision.enabled);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                limelight = new Limelight(new LimelightIOReal(), drivetrain);
                limelight.useLimelight(false);
                break;

            default:
                // Replayed robot, disable IO implementations
                limelight = new Limelight(new LimelightIO() {
                }, drivetrain);
                limelight.useLimelight(RobotConstants.Vision.enabled);
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
            drivetrain.seedFieldRelative(new Pose2d(MathUtils.translation.zero, MathUtils.rotation.half));
        } else {
            // Start at blue origina
            DataLogManager.log("INITIAL POSITION: BLUE ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(MathUtils.translation.zero, MathUtils.rotation.zero));
        }
    }

    public void resetPoseForward(Alliance alliance) {
        Pose2d currentPose = drivetrain.getState().Pose;
        if (alliance == Alliance.Red) { 
            // Start at red origin, but facing blue alliance wall
            DataLogManager.log("RESETTING FORWARD: RED ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(currentPose.getX(), currentPose.getY(), MathUtils.rotation.half));
        } else {
            // Start at blue origina
            DataLogManager.log("RESETTING FORWARD: BLUE ALLIANCE");
            drivetrain.seedFieldRelative(new Pose2d(currentPose.getX(), currentPose.getY(), MathUtils.rotation.zero));
        }
    }
}
