/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.conveyorPID;
import edu.wpi.first.wpilibj.Timer;

public class ConveyorIn extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final conveyorPID m_conveyor;
  private boolean ender;
  private Timer timer;
  private boolean fifthBall;
  double target = -9.0;
  double tolerance = 0.1;
  int modenum = 2;
  private Auto m_auto;
  private Intake m_intake;
  private Drivetrain m_drivetrain = new Drivetrain();
  private Sensors m_sensor;
  /**
   * Creates a new ConveyorIn.
   */
  public ConveyorIn(conveyorPID conveyor, Sensors sensor) {
    m_conveyor = conveyor;
    m_sensor = sensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
    addRequirements(sensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ender = false;
    fifthBall = false;
    m_conveyor.enable();
    m_conveyor.setIntake(m_conveyor.down);
    System.out.println("Starting Step 3!");
    System.out.println("Runs is: " + m_conveyor.runs);
    timer = new Timer();
    m_auto = new Auto(m_drivetrain, m_conveyor, m_sensor);
    m_intake = new Intake(m_conveyor, m_sensor);
    if (m_conveyor.lastBall) target = -1.0;
    else target = -8.5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_conveyor.getMeasurement() <= (target + tolerance) || m_conveyor.mode != modenum){
      ender = true; 
      if(m_conveyor.mode != modenum){
        System.out.println("Step 3 will finish because my mode is not 3! My mode is: " + m_conveyor.mode);
      } else if(m_conveyor.getMeasurement() <= (target + tolerance)){
        System.out.println("Step 3 will finish because I have reached my target!");
      }
    } else{
        m_conveyor.setIntakeMotors(0.0);
        m_conveyor.setSetpoint(target);
        System.out.println("I should have moved the conveyor out! I am still not close enough to my target! My error is: " + (9 + m_conveyor.getMeasurement()));
      }
    }

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) {
  if(m_conveyor.mode == modenum && !interrupted){
    m_conveyor.mode = 0; 
    if (m_conveyor.lastBall) {
      m_conveyor.mode = 4;
      m_conveyor.lastBall = false;
    }
    m_conveyor.resetEncoder();
    m_conveyor.setSetpoint(0.0);
    System.out.println("My mode is now: " + m_conveyor.mode + "!");
  }
  System.out.println("Step 3 has finished!");
  if(m_conveyor.runs == 1){
    m_auto.schedule();
    m_conveyor.runs = 2;
    System.out.println("Run was 1, so Auto was scheduled! Runs is: " + m_conveyor.runs);
  }else if(m_conveyor.runs == 0){
    m_conveyor.runs = 1;
    m_intake.schedule();
    System.out.println("Run was 0, so Intake was scheduled! Run is now: " + m_conveyor.runs);
  }
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ender;
  }
}
