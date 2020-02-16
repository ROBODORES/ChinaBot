/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FrontIntake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
/*public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  //private final Climber m_climber = new Climber();
  //private final FrontIntake m_frontIntake = new FrontIntake();
  //private final Conveyor m_conveyor = new Conveyor();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //private final Test m_test = new Test(m_conveyor, m_frontIntake);
  //public static Deploy deploy = new Deploy();

  public static Joystick controller;
  public static JoystickButton aButton;
  public static JoystickButton bButton;
  public static JoystickButton xButton;
  public static JoystickButton yButton;
  public static JoystickButton leftBumperButton;
  public static JoystickButton rightBumperButton;
  public static JoystickButton leftMiniButton;
  public static JoystickButton rightMiniButton;
  public static JoystickButton leftStickButton;
  public static JoystickButton rightStickButton;

  public static XboxController xController;
  *//**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
  
 /* public RobotContainer() {
    // Configure the button bindings
    controller = new Joystick(0);
    /*aButton = new JoystickButton(controller, 1);
    bButton = new JoystickButton(controller, 2);
    xButton = new JoystickButton(controller, 3);
    yButton = new JoystickButton(controller, 4);
    leftBumperButton = new JoystickButton(controller, 5);*/
    //rightBumperButton = new JoystickButton(controller, 6);
    /*leftMiniButton = new JoystickButton(controller, 7);
    rightMiniButton = new JoystickButton(controller, 8);
    leftStickButton = new JoystickButton(controller, 9);
    rightStickButton = new JoystickButton(controller, 10);
    xController = new XboxController(0);*/

   /* configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
 /* private void configureButtonBindings() {
    //new JoystickButton(controller, 6).whileHeld(m_test);//anger
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /* public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
      // Create a voltage constraint to ensure we don't accelerate too fast

   
      var autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                         DriveConstants.kvVoltSecondsPerMeter,
                                         DriveConstants.kaVoltSecondsSquaredPerMeter),
                                          DriveConstants.kDriveKinematics,10);
  
      // Create config for trajectory
      TrajectoryConfig config =
          new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(DriveConstants.kDriveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint);
  
      // An example trajectory to follow.  All units in meters.
      String trajectoryJSON = "paths/pathweaver.json";
      Path trajectoryPath;
      Trajectory exampleTrajectory;
      try {
        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // An example trajectory to follow.  All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
              new Translation2d(1, 1),
              new Translation2d(2, -1)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config
        );
      }
  
      RamseteCommand ramseteCommand = new RamseteCommand(
          exampleTrajectory,
          m_drivetrain::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                     DriveConstants.kvVoltSecondsPerMeter,
                                     DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          m_drivetrain::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrain::tankDriveVolts,
          m_drivetrain
      );
  
      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  
    return m_autoCommand;
  }
}
*/
