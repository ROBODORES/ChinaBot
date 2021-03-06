/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Extend extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Climber m_climber;

  /**
   * Creates a new Extend.
   */
  public Extend(final Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*double target = 3.0;
    m_climber.setClimbMotors(0.7);
    if(m_climber.getSEncoderRaw() >= target){
      m_climber.setClimbMotors(0.0);
      m_climber.extend();
    }*/
    m_climber.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
