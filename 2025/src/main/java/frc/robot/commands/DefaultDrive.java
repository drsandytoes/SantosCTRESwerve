package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.MathUtils;

@Logged(name = "DefaultDriveCommand")
public class DefaultDrive extends Command {
    @NotLogged protected CommandSwerveDrivetrain drivetrain;
    protected DoubleSupplier xSupplier, ySupplier, thetaSupplier;

    @NotLogged private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(RobotConstants.Drivetrain.MaxSpeed * RobotConstants.Driver.deadband)
            .withRotationalDeadband(RobotConstants.Drivetrain.MaxAngularRate * RobotConstants.Driver.deadband)                                                                                                                // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @NotLogged private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(RobotConstants.Drivetrain.MaxSpeed * RobotConstants.Driver.deadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    // Keep track of the last know heading/orientation. If the driver isn't
    // providing rotational input, we want
    // to try to maintain the orientation of the robot when the driver last stopped
    // providing rotational input.

    protected Rotation2d savedHeading;
    protected boolean driverIsRotatingRobot = true;
    protected boolean useHeadingController = RobotConstants.Driver.useHeadingController;

    /**
     * Drive command that uses joystick input (corrected for FRC coordinates).
     * 
     * @param xSupplier     Raw input value supplier for forward (FRC X) direction
     * @param ySupplier     Raw input value supplier for left (FRC Y) direction
     * @param thetaSupplier Raw input value supplier for CCW (FRC theta) rotation
     * @param drivetrain    The CTRE swerve drivetrain subsystem
     */
    public DefaultDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier,
            CommandSwerveDrivetrain drivetrain) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;
        savedHeading = new Rotation2d();

        // Set the PID constants for the built-in heading controller
        driveAtAngle.HeadingController.setPID(RobotConstants.Drivetrain.HeadingController.kP,
                RobotConstants.Drivetrain.HeadingController.kI,
                RobotConstants.Drivetrain.HeadingController.kD);
        driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        Shuffleboard.getTab("Driver").addDouble("Saved Heading", () -> savedHeading.getDegrees());
    }

    @Override
    public void initialize() {
        savedHeading = drivetrain.getState().Pose.getRotation();
        driverIsRotatingRobot = true;
    }

    @Override
    public void execute() {
        // Apply deadband to rotation here so we make the same decision the drive system
        // will
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double theta = thetaSupplier.getAsDouble();

        double deadbandedTheta = MathUtil.applyDeadband(theta, RobotConstants.Driver.deadband);
        double xSpeed = x * RobotConstants.Drivetrain.MaxSpeed;
        double ySpeed = y * RobotConstants.Drivetrain.MaxSpeed;
        double thetaSpeed = theta * RobotConstants.Drivetrain.MaxAngularRate;

        if (Math.abs(deadbandedTheta) < 0.001) {
            // Driver is not rotating the robot. If they were the last time this was called,
            // latch the current heading
            // as the one desired by the driver. We might want to delay the latch to account
            // for intertia. When the driver
            // stops rotating, the robot would normally still rotate for a short period of
            // time, and it may feel weird
            // for that to be cut off. It also may make it difficult to make fine
            // adjustments to the heading, so there's
            // opportunity to improve this based on driver feedback.
            if (driverIsRotatingRobot) {
                double currentAngleRad = drivetrain.getState().Pose.getRotation().getRadians();
                currentAngleRad = MathUtil.angleModulus(currentAngleRad);
                Rotation2d headingToLock = Rotation2d.fromRadians(currentAngleRad);

                // Pose is blue-alliance field centric standard. But if the drive train is
                // operator-relative and we're on the red alliance, we need to rotate the 
                // heading 180 degrees. We do this here before saving it so we don't need to do
                // the math repeatedly.
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    if (alliance.get() == DriverStation.Alliance.Red) {
                        headingToLock = headingToLock.rotateBy(MathUtils.rotation.kHalf);
                    }
                }
                savedHeading = headingToLock;
            }
            driverIsRotatingRobot = false;
        } else {
            driverIsRotatingRobot = true;
        }

        if (!useHeadingController || driverIsRotatingRobot) {
            // There is rotational input (or we're not using the heading controller)

            // Pass all of the speeds to the drive train
            drivetrain.setControl(drive.withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(thetaSpeed));
        } else {
            // Pass x/y speeds, but use the saved heading as the direction. If the robot
            // bumps into something, it will
            // try to go back to the saved angle.
            drivetrain.setControl(driveAtAngle.withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withTargetDirection(savedHeading));
        }
    }
}
