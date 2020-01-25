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
  WPI_VictorSPX topConveyorMotor;
  WPI_VictorSPX bottomConveyorMotor;
  Solenoid stopper;
  public static boolean close = false;
  public static boolean open = true;

  Solenoid intakeSol;
  WPI_VictorSPX leftIntakeMotor;
  WPI_VictorSPX rightIntakeMotor;
  public static boolean up = false;
  public static boolean down = true;
  
  public Conveyor(){
    topConveyorMotor = new WPI_VictorSPX(Constants.topMotor);
    bottomConveyorMotor = new WPI_VictorSPX(Constants.bottomMotor);
    stopper = new Solenoid(Constants.stopper);

    intakeSol = new Solenoid(Constants.intakeSol);
    leftIntakeMotor = new WPI_VictorSPX(Constants.leftIntakeMotor);
    rightIntakeMotor = new WPI_VictorSPX(Constants.rightIntakeMotor);
  }

  public void setConveyorMotors(double speed){
    topConveyorMotor.set(speed);
    bottomConveyorMotor.set(speed);
  }

  public void setStopper(boolean state){
    stopper.set(state);
  }

  public void setIntake(boolean state){
    intakeSol.set(state);
  }

  public void setIntakeMotors(double speed){
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }
}
