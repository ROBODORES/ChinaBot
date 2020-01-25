/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Climber m_climber = new Climber();
  private final Conveyor m_conveyor = new Conveyor();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //private final Test m_test = new Test(m_conveyor, m_frontIntake);
  //public static Deploy deploy = new Deploy();

  /*public static Joystick controller;
  public static JoystickButton aButton;
  public static JoystickButton bButton;
  public static JoystickButton xButton;
  public static JoystickButton yButton;
  public static JoystickButton leftBumperButton;
  public static JoystickButton rightBumperButton;
  public static JoystickButton leftMiniButton;
  public static JoystickButton rightMiniButton;
  public static JoystickButton leftStickButton;
  public static JoystickButton rightStickButton;*/

  public static XboxController m_xController;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    /*controller = new Joystick(0);
    aButton = new JoystickButton(controller, 1);
    bButton = new JoystickButton(controller, 2);
    xButton = new JoystickButton(controller, 3);
    yButton = new JoystickButton(controller, 4);
    leftBumperButton = new JoystickButton(controller, 5);
    rightBumperButton = new JoystickButton(controller, 6);
    leftMiniButton = new JoystickButton(controller, 7);
    rightMiniButton = new JoystickButton(controller, 8);
    leftStickButton = new JoystickButton(controller, 9);
    rightStickButton = new JoystickButton(controller, 10);
    xController = new XboxController(0);*/

    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_xController, Button.kBumperRight.value).whenPressed(new Intake(m_conveyor)); //Intake
    new JoystickButton(m_xController, Button.kX.value).whenPressed(new Deploy(m_conveyor));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
