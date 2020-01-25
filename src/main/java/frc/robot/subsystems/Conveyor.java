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
import edu.wpi.first.wpilibj.SpeedControllerGroup;

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
  SpeedControllerGroup conveyorMotors;

  Solenoid intakeSol;
  WPI_VictorSPX leftIntakeMotor;
  WPI_VictorSPX rightIntakeMotor;
  public static boolean up = false;
  public static boolean down = true;
  SpeedControllerGroup intakeMotors;
  
  public Conveyor(){
    topConveyorMotor = new WPI_VictorSPX(Constants.topMotor);
    bottomConveyorMotor = new WPI_VictorSPX(Constants.bottomMotor);
    bottomConveyorMotor.setInverted(true);
    stopper = new Solenoid(Constants.stopper);
    conveyorMotors = new SpeedControllerGroup(topConveyorMotor, bottomConveyorMotor);

    intakeSol = new Solenoid(Constants.intakeSol);
    leftIntakeMotor = new WPI_VictorSPX(Constants.leftIntakeMotor);
    leftIntakeMotor.setInverted(true);
    rightIntakeMotor = new WPI_VictorSPX(Constants.rightIntakeMotor);
    intakeMotors = new SpeedControllerGroup(leftIntakeMotor, rightIntakeMotor);
  }

  public void setConveyorMotors(double speed){
    conveyorMotors.set(speed);
  }

  public void setStopper(boolean state){
    stopper.set(state);
  }

  public void setIntake(boolean state){
    intakeSol.set(state);
  }

  public void setIntakeMotors(double speed){
    intakeMotors.set(speed);
  }
}
