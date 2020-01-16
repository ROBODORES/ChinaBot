/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Conveyor extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_VictorSPX topMotor;
  WPI_VictorSPX bottomMotor;
  Solenoid stopper;
  public static boolean close = false;
  public static boolean open = true;
  
  public Conveyor(){
    topMotor = new WPI_VictorSPX(Constants.topMotor);
    bottomMotor = new WPI_VictorSPX(Constants.bottomMotor);
    stopper = new Solenoid(Constants.stopper);
  }

  public void setMotors(double speed){
    topMotor.set(speed);
    bottomMotor.set(speed);
  }

  public void setStopper(boolean state){
    stopper.set(state);
  }
}
