/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Intake extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Conveyor m_conveyor;

  /**
   * Creates a new Test.
   */
  public Intake(final Conveyor conveyor) {
    m_conveyor = conveyor;
    // here to declare subsystem dependencies.
    addRequirements(conveyor);
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_conveyor.setStopper(m_conveyor.close);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_conveyor.setIntake(m_conveyor.down);
    //m_conveyor.setConveyorMotors(-1.0);
    //m_conveyor.conveyorIntake();
    m_conveyor.setConveyorMotors(RobotContainer.m_mechController.getRawAxis(1), RobotContainer.m_mechController.getRawAxis(5));
    m_conveyor.setIntakeMotors(0.2);
    System.out.println("Top Conveyor Speed: " + RobotContainer.m_mechController.getRawAxis(1) + ", Bottom Conveyor Speed: " + RobotContainer.m_mechController.getRawAxis(5));
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}
