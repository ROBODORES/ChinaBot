/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveConstants;

public class PathFinder extends SubsystemBase {

  public  DifferentialDrive m_Drive;

  public final Encoder m_leftEncoder;
  public final Encoder m_rightEncoder;
 public final AHRS m_navx = new AHRS(SPI.Port.kMXP);
 public  final int[] LeftEncoderPorts = new int[]{0, 1};
 public  final int[] RightEncoderPorts = new int[]{2, 3};
 private final SpeedControllerGroup m_leftMotors =
 new SpeedControllerGroup(new WPI_TalonSRX(DriveConstants.LeftMotor1Port),
                          new WPI_TalonSRX(DriveConstants.LeftMotor2Port));

// The motors on the right side of the drive.
private final SpeedControllerGroup m_rightMotors =
 new SpeedControllerGroup(new WPI_TalonSRX(DriveConstants.RightMotor1Port),
                          new WPI_TalonSRX(DriveConstants.RightMotor2Port));


  // Odometry class for tracking robot pose
  private  DifferentialDriveOdometry m_odometry;

  // Note that the angle from the NavX and all implementors of wpilib Gyro
  // must be negated because getAngle returns a clockwise positive angle
  AHRS navx = new AHRS(SPI.Port.kMXP);
  { 
        m_leftEncoder = new Encoder(0, 1);
        m_leftEncoder.setReverseDirection(true);
        m_leftEncoder.setDistancePerPulse(DriveConstants.LeftMetersPerPulse);
   
       // The right-side drive encoder
        m_rightEncoder = new Encoder(2, 3);
       m_rightEncoder.setDistancePerPulse(DriveConstants.RightMetersPerPulse);
       m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
      }
   
    

    // The left-side drive encoder
  
    

    

  public PathFinder() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.EncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.EncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                      m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_Drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_Drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    // The left-side drive encoder
    final Encoder m_leftEncoder = new Encoder(DriveConstants.LeftEncoderPorts[0],
    DriveConstants.LeftEncoderPorts[1], DriveConstants.LeftEncoderReversed) ; 
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    final Encoder m_rightEncoder = new Encoder(DriveConstants.RightEncoderPorts[2],
        DriveConstants.RightEncoderPorts[3], DriveConstants.RightEncoderReversed);
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_Drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    
    

    
    m_navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_navx.getAngle(), 360) * (DriveConstants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate() * (DriveConstants.GyroReversed ? -1.0 : 1.0);

  }
 
  }
