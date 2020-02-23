/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyorPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.conveyorPID;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class FrontIntaking extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final conveyorPID m_conveyor;
  public boolean intaking;

  /**
   * Creates a new Test.
   */
  public FrontIntaking(final conveyorPID conveyor) {
    m_conveyor = conveyor;
    // here to declare subsystem dependencies.
    addRequirements(conveyor);
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_conveyor.enable(); 
    m_conveyor.setIntake(m_conveyor.down);
    //m_conveyor.setStopper(m_conveyor.close);
    intaking = false;
    System.out.println("At the start!");
    if (m_conveyor.getSensorInput2()) {
      m_conveyor.lastBall = true;
      System.out.println("Last Ball");
    } else {
      m_conveyor.lastBall = false;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(m_conveyor.checker() == true || m_conveyor.mode != 0){
      if(m_conveyor.mode != 0){
        System.out.println("Step 1 has finished because my mode isnt 0. My mode is: " + m_conveyor.mode);
      }
      intaking = true;
    } else{
      m_conveyor.setIntakeMotors(0.3);
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if(m_conveyor.mode == 0 && !interrupted){
      m_conveyor.mode = 1; 
      m_conveyor.resetEncoder();
      m_conveyor.setSetpoint(0.0);
      System.out.println("My mode is now: " + m_conveyor.mode);
    }else{
      System.out.println("Finishing step 0");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return intaking; 
  }
}
