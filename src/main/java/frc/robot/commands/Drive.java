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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  /**
   * Creates a new Test.
   */
  public Drive(final Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(-0.75 * RobotContainer.m_driverController.getRawAxis(1), 0.75 * RobotContainer.m_driverController.getRawAxis(4));
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
