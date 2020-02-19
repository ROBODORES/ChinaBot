/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyorPID;

public class ConveyorIn extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final conveyorPID m_conveyor;
  private boolean ender = false;
  /**
   * Creates a new ConveyorIn.
   */
  public ConveyorIn(conveyorPID conveyor) {
    m_conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyor.enable();
    m_conveyor.setIntake(m_conveyor.down);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_conveyor.getMeasurement() <= -8.99 || m_conveyor.mode != 2){
      ender = true; 
    } else{
      m_conveyor.setIntakeMotors(0.0);
      m_conveyor.setSetpoint(-9);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) {
  if(m_conveyor.mode == 2){
    m_conveyor.mode = 0; 
    m_conveyor.resetEncoder();
    m_conveyor.setSetpoint(0.0);
  }
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ender;
  }
}
