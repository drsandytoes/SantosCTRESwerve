package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DefaultDrive extends Command {
  protected CommandSwerveDrivetrain drivetrain;
  protected DoubleSupplier xSupplier, ySupplier, thetaSupplier;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(RobotConstants.Drivetrain.MaxSpeed * RobotConstants.Driver.deadband)
    .withRotationalDeadband(RobotConstants.Drivetrain.MaxAngularRate * RobotConstants.Driver.deadband) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(RobotConstants.Drivetrain.MaxSpeed * RobotConstants.Driver.deadband)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // Keep track of the last know heading/orientation. If the driver isn't providing rotational input, we want
  // to try to maintain the orientation of the robot when the driver last stopped providing rotational input.
  protected Rotation2d savedHeading;
  protected boolean driverIsRotatingRobot = true;
  protected boolean useHeadingController = true;

  /**
   * Drive command that uses joystick input (corrected for FRC coordinates). 
   * @param xSupplier Raw input value supplier for forward (FRC X) direction
   * @param ySupplier Raw input value supplier for left (FRC Y) direction
   * @param thetaSupplier Raw input value supplier for CCW (FRC theta) rotation
   * @param drivetrain The CTRE swerve drivetrain subsystem 
   */
  public DefaultDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, CommandSwerveDrivetrain drivetrain) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;

    driveAtAngle.HeadingController.setPID(RobotConstants.Drivetrain.HeadingController.kP, 
      RobotConstants.Drivetrain.HeadingController.kI, 
      RobotConstants.Drivetrain.HeadingController.kD);

    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    savedHeading = drivetrain.getState().Pose.getRotation();
    driverIsRotatingRobot = true;
  }

  @Override
  public void execute() {
    // Apply deadband to rotation here so we make the same decision the drive system will
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    double theta = thetaSupplier.getAsDouble();

    double deadbandedTheta = MathUtil.applyDeadband(theta, RobotConstants.Driver.deadband);
    double xSpeed = x * RobotConstants.Drivetrain.MaxSpeed;
    double ySpeed = y * RobotConstants.Drivetrain.MaxSpeed;
    double thetaSpeed = theta * RobotConstants.Drivetrain.MaxAngularRate;

    if (!useHeadingController || Math.abs(deadbandedTheta) > 0.001) {
        // There is rotational input (or we're not using the heading controller)
        driverIsRotatingRobot = true;

        // Pass all of the speeds to the drive train
        drivetrain.setControl(drive.withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withRotationalRate(thetaSpeed));
    } else {
        // Driver is not rotating the robot. If they were the last time this was called, latch the current heading
        // as the one desired by the driver.
        if (driverIsRotatingRobot) {
            savedHeading = drivetrain.getState().Pose.getRotation();
        }
        driverIsRotatingRobot = false;

        // Pass x/y speeds, but use the saved heading as the direction. If the robot bumps into something, it will
        // try to go back to the saved angle.
        drivetrain.setControl(driveAtAngle.withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withTargetDirection(savedHeading));
    }
  }
}
