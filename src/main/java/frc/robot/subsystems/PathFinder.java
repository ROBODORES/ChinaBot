/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/*import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/*public class PathFinder extends SubsystemBase {
  private final SpeedController m_leftmotors, m_rightmotors;

  private final DifferentialDrive m_dDrive;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;*/

  // Note that the angle from the NavX and all implementors of wpilib Gyro
  // must be negated because getAngle returns a clockwise positive angle
  /*AHRS navx = new AHRS(SPI.Port.kMXP);
 final String trajectoryJSON = "paths/YourPath.wpilib.json";{
  }{
  try {
    final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    final Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (final IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }
  final String trajectoryJSON = "paths/YourPath.wpilib.json"; }*/

  /*final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 0;
    public static final int kRightMotor2Port = 1;
    public static final DigitalSource[] LeftEncoderPort = null;
    public static final DigitalSource LeftEncoderReversed = null;
    public static final DigitalSource[] RightEncoderPorts = null;
    public static final DigitalSource RightEncoderReversed = null;
  }*/
  // Supplier<Double> gyroAngleRadians;

  /**
   * Creates a new VictorSP_DriveTrain.
   */
  
    // gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());
    

    // The left-side drive encoder
    /*final int[] LeftEncoderPorts = new int[]{0, 1};
    final int[] RightEncoderPorts = new int[]{2, 3};
    final boolean LeftEncoderReversed = true;
    final boolean RightEncoderReversed = false;}*/
    


   /* m_leftEncoder = new Encoder(DriveConstants.LeftEncoderPort[0], DriveConstants.LeftEncoderPort[1],
     DriveConstants.LeftEncoderReversed);
    // m_leftEncoder = new Encoder(0, 1);
    // m_leftEncoder.setReverseDirection(true);
    //m_leftEncoder.setDistancePerPulse(DriveConstants.LeftMetersPerPulse);
    // leftEncoderPosition = leftEncoder::getDistance;
    // leftEncoderRate = leftEncoder::getRate;

    // The right-side drive encoder
   /* m_rightEncoder = new Encoder(DriveConstants.RightEncoderPorts[0], DriveConstants.RightEncoderPorts[1],
                  DriveConstants.RightEncoderReversed);
    // m_rightEncoder = new Encoder(2, 3);
   // m_rightEncoder.setDistancePerPulse(DriveConstants.RightMetersPerPulse);
    // rightEncoderPosition = rightEncoder::getDistance;
    // rightEncoderRate = rightEncoder::getRate;

    //resetEncoders();
    //m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    //m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
    // m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  /*public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
*/
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
 /* public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  /*public void resetOdometry(final Pose2d pose) {
    resetEncoders();
   // m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
 /* public void arcadeDrive(final double fwd, final double rot) {
    m_dDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  /*public void tankDriveVolts(final double leftVolts, final double rightVolts) {
    m_leftmotors.setVoltage(leftVolts);
    m_rightmotors.setVoltage(-rightVolts);
  }*/

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
 /* public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }*/

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
 /* public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }*/

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  /*public Encoder getRightEncoder() {
    return m_rightEncoder;
  }*/

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
 /* public void setMaxOutput(final double maxOutput) {
    m_dDrive.setMaxOutput(maxOutput);
  }*/


  /**
   * Zeroes the heading of the robot.
   */
 /* public void zeroHeading() {
    navx.reset();
  }*/

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  //public double getHeading() {
   // return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  /*public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
{


}}
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
 // public double getTurnRate() {
    
   // return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
   







