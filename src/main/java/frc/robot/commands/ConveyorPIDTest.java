/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyorPID;

public class ConveyorPIDTest extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final conveyorPID m_conveyor;
  double setpoint = -10.0;
  double tolerance = 0.2;
  /**
   * Creates a new ConveyorPIDTest.
   */
  public ConveyorPIDTest(conveyorPID conveyor) {
    m_conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyor.resetEncoder();
    m_conveyor.enable();
    m_conveyor.setIntake(m_conveyor.down);
    m_conveyor.setSetpoint(-10.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.setIntakeMotors(0.3);
    System.out.println(m_conveyor.atSetpoint(tolerance, setpoint));
    System.out.println("Error: " + (10.0 + m_conveyor.getMeasurement()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.setIntakeMotors(0.0);
    //m_conveyor.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_conveyor.atSetpoint(tolerance, setpoint);
  }
}
