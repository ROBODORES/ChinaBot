/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyorPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import edu.wpi.first.wpilibj.Timer;

public class DriveBack extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain m_drivetrain;
  private Sensors m_sensor;
  private Timer m_timer = new Timer();
  private boolean ender;
  private Intake m_intake;
  private conveyorPID m_conveyor;
  /**
   * Creates a new RunAuto.
   */
  public DriveBack(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ender = false;
    System.out.println("First Step Starting!");
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(0.3, 0.0);
    System.out.println("I should be driving backwards!");
    if(m_conveyor.runs == 2){
      m_drivetrain.arcadeDrive(0.0, 0.0);
      System.out.println("I should have stopped!");
      ender = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ender = false;
    System.out.println("First Step Done!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ender;
  }
}