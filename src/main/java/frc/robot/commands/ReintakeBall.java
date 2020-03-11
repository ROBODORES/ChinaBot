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

public class ReintakeBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final conveyorPID m_conveyor;
  private Timer time = new Timer();
  boolean ender = false;
  /**
   * Creates a new ReintakeBall.
   */
  public ReintakeBall(conveyorPID conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = conveyor; 
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    ender = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (time.get() >= 0.1 || m_conveyor.mode != 2) {
        m_conveyor.setIntakeMotors(0.0);
        ender = true;
      } else {
        m_conveyor.setIntakeMotors(0.3);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    if(m_conveyor.mode == 2 && !interrupted){
      m_conveyor.mode = 3; 
      m_conveyor.resetEncoder();
      m_conveyor.setSetpoint(0.0);
      System.out.println("Step 2 Finished! My mode is now: " + m_conveyor.mode);
    }
    System.out.println("Step 2 Finished!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ender;
  }
}
