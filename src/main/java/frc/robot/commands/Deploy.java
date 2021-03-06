/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;

public class Deploy extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Conveyor m_conveyor;

  /**
   * Creates a new Test.
   */
  public Deploy(final Conveyor conveyor) {
    m_conveyor = conveyor;
    // here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //m_conveyor.setStopper(m_conveyor.open);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_conveyor.setIntake(m_conveyor.up);
    //m_conveyor.setConveyorMotors(1.0);
    m_conveyor.conveyorDeploy();
    m_conveyor.setIntakeMotors(0.0);
    //System.out.println("Feedback: " + m_conveyor.getConveyorEncoderDistance());
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
