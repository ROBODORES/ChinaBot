/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Conveyor;

public class Stop extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Conveyor m_conveyor;

  /**
   * Creates a new Test.
   */
  public Stop(final Conveyor conveyor) {
    m_conveyor = conveyor;
    // here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_conveyor.setStopper(m_conveyor.close);
    m_conveyor.conveyorStop();
    m_conveyor.setIntakeMotors(0.0);
    m_conveyor.setIntake(m_conveyor.up);
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
