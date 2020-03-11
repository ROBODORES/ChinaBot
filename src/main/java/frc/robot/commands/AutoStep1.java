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

public class AutoStep1 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain m_drivetrain;
  private Sensors m_sensor;
  private boolean hitWall;
  private boolean ender;
  /**
   * Creates a new RunAuto.
   */
  public AutoStep1(Drivetrain drivetrain, Sensors sensor) {
    m_drivetrain = drivetrain;
    m_sensor = sensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(sensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hitWall = false;
    ender = false;
    System.out.println("First Step Starting!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(-0.4, 0.0);
    System.out.println("I should be driving!");
    if(m_sensor.getUSSensorVoltage() < 0.3){
      hitWall = true;
    }
    if(hitWall){
      m_drivetrain.arcadeDrive(0.0, 0.0);
      System.out.println("I should have stopped!");
      ender = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ender = false;
    hitWall = false;
    System.out.println("First Step Done!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ender;
  }
}
