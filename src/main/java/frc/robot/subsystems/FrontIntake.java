/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class FrontIntake extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Solenoid intakeSol;
  WPI_VictorSPX leftIntakeMotor;
  WPI_VictorSPX rightIntakeMotor;
  public static boolean up = false;
  public static boolean down = true;


  public FrontIntake(){
    intakeSol = new Solenoid(Constants.intakeSol);
    leftIntakeMotor = new WPI_VictorSPX(Constants.leftIntakeMotor);
    rightIntakeMotor = new WPI_VictorSPX(Constants.rightIntakeMotor);
  }

  public void setArm(boolean state){
    intakeSol.set(state);
  }

  public void setMotors(double speed){
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }
}
