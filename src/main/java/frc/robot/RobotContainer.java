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
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

/*
   __ __   ___     ______    ______
  / // /  |__ \   / ____/   / ____/
 / // /_  __/ /  /___ \    /___ \  
/__  __/ / __/  ____/ /   ____/ /  
  /_/   /____/ /_____/   /_____/ 
  _____   _                ____            _                   _                             
 |_   _| | |__     ___    |  _ \    ___   | |__     ___     __| |   ___    _ __    ___   ___ 
   | |   | '_ \   / _ \   | |_) |  / _ \  | '_ \   / _ \   / _` |  / _ \  | '__|  / _ \ / __|
   | |   | | | | |  __/   |  _ <  | (_) | | |_) | | (_) | | (_| | | (_) | | |    |  __/ \__ \
   |_|   |_| |_|  \___|   |_| \_\  \___/  |_.__/   \___/   \__,_|  \___/  |_|     \___| |___/
                                                                                             
*/

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
  private final Conveyor m_conveyor = new Conveyor();
  private final Climber m_climber = new Climber();
  //private final LEDs m_LEDs = new LEDs();
  private final UltrasonicSensor m_sensor = new UltrasonicSensor();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final RunAuto m_auto = new RunAuto(m_drivetrain, m_sensor, m_conveyor);

  public static XboxController m_driverController = new XboxController(0);
  public static XboxController m_mechController = new XboxController(1);

  //First Controller Setup
  private static JoystickButton intakeButton = new JoystickButton(m_mechController, Button.kBumperRight.value);
  private static JoystickButton outtakeButton = new JoystickButton(m_mechController, Button.kBumperLeft.value);
  private static JoystickButton deployButton = new JoystickButton(m_mechController, Button.kX.value);
  private static JoystickButton climberUpButton = new JoystickButton(m_mechController, Button.kY.value);
  private static JoystickButton climberDownButton = new JoystickButton(m_mechController, Button.kA.value);
  private static JoystickButton liftButton = new JoystickButton(m_mechController, Button.kB.value);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain, m_sensor));
    m_climber.setDefaultCommand(new Retract(m_climber));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeButton.whenPressed(new Intake(m_conveyor)); //Intake
    intakeButton.whenReleased(new Stop(m_conveyor));
    outtakeButton.whenPressed(new Outtake(m_conveyor)); //Outtake
    outtakeButton.whenReleased(new Stop(m_conveyor));
    deployButton.whenPressed(new Deploy(m_conveyor)); //Deploy
    deployButton.whenReleased(new Stop(m_conveyor));
    climberUpButton.whenPressed(new Extend(m_climber));
    climberDownButton.whenPressed(new Retract(m_climber));
    liftButton.whileHeld(new Climb(m_climber));
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
