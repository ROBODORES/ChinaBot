/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyorPID;

public class AutoStep2 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private conveyorPID m_conveyor;
  private Timer m_timer = new Timer();
  private boolean ender;
  /**
   * Creates a new RunAuto2.
   */
  public AutoStep2(conveyorPID conveyor) {
    m_conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    ender = false;
    System.out.println("Second Step Starting!");
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.conveyorDeploy();
    System.out.println("Conveyor should be deploying!");
    if(m_timer.get() > 3.0) {
      ender = true;
      System.out.println("Conveyor should have ran for 3 seconds!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ender = false;
    System.out.println("Step 2 finished!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ender;
  }
}
