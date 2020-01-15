/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Solenoid climbSol1;
  Solenoid climbSol2;
  Solenoid climbSol3;
  Solenoid climbSol4;

  public Climber(){
    climbSol1 = new Solenoid(Constants.climberSol1);
    climbSol2 = new Solenoid(Constants.climberSol2);
    climbSol3 = new Solenoid(Constants.climberSol3);
    climbSol4 = new Solenoid(Constants.climberSol4);
  }

  public void firstExtension(){
    climbSol1.set(true);
    climbSol2.set(true);
  }

  public void secondExtension(){
    climbSol3.set(true);
    climbSol4.set(true);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
