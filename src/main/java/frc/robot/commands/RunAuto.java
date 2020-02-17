/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.UltrasonicSensor;
import edu.wpi.first.wpilibj.Timer;

public class RunAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain m_drivetrain;
  private UltrasonicSensor m_sensor;
  private Conveyor m_conveyor;
  private double targetVolts = 0.5;
  //private double wiggleRoom = 0.1;
  private Timer m_timer;
  /**
   * Creates a new RunAuto.
   */
  public RunAuto(Drivetrain drivetrain, UltrasonicSensor sensor, Conveyor conveyor) {
    m_drivetrain = drivetrain;
    m_sensor = sensor;
    m_conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(sensor);
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(0.2, 0.0);
    m_sensor.readSensorInput();
    if(m_sensor.getVolts() < (targetVolts)){
      m_drivetrain.arcadeDrive(0.0, 0.0);
      m_timer.start();
      m_conveyor.conveyorDeploy();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.get() >= 5){
      m_conveyor.setConveyorMotors(0.0, 0.0);
      return true;
    }
    return false;
  }
}
