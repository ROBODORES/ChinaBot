/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class Conveyor extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  VictorSPX topConveyorMotor;
  VictorSPX bottomConveyorMotor;
  Solenoid stopper;
  public static boolean close = false;
  public static boolean open = true;
  SpeedControllerGroup conveyorMotors;

  Solenoid intakeSol;
  VictorSPX leftIntakeMotor;
  VictorSPX rightIntakeMotor;
  public static boolean up = false;
  public static boolean down = true;
  SpeedControllerGroup intakeMotors;

  Encoder conveyorEncoder;
  DigitalInput infraredSensor;

  Timer timer;
  
  public Conveyor(){
    topConveyorMotor = new VictorSPX(Constants.topMotor);
    bottomConveyorMotor = new VictorSPX(Constants.bottomMotor);
    bottomConveyorMotor.setInverted(false);
    stopper = new Solenoid(Constants.stopper);

    intakeSol = new Solenoid(Constants.intakeSol);
    leftIntakeMotor = new VictorSPX(Constants.leftIntakeMotor);
    leftIntakeMotor.setInverted(false);
    rightIntakeMotor = new VictorSPX(Constants.rightIntakeMotor);

    conveyorEncoder = new Encoder(Constants.conveyorEncoderA, Constants.conveyorEncoderB);

    timer = new Timer();
    infraredSensor= new DigitalInput(0);

    //bottomConveyorMotor.follow(topConveyorMotor);
    leftIntakeMotor.follow(rightIntakeMotor);

    timer.start();
  }

  public void setConveyorMotors(double speed1, double speed2){
    topConveyorMotor.set(ControlMode.PercentOutput, speed1);
    bottomConveyorMotor.set(ControlMode.PercentOutput, speed2);
  }

  public void conveyorReverseIntake(){
    topConveyorMotor.set(ControlMode.PercentOutput, 0.5);
    bottomConveyorMotor.set(ControlMode.PercentOutput, 0.5);
  }
  public void conveyorIntake(){
    topConveyorMotor.set(ControlMode.PercentOutput, -0.7);
    bottomConveyorMotor.set(ControlMode.PercentOutput, -0.7);
  }

  public void conveyorExtake(){
    topConveyorMotor.set(ControlMode.PercentOutput, 1.0);
    bottomConveyorMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void conveyorDeploy(){
    topConveyorMotor.set(ControlMode.PercentOutput, -1.0);
    bottomConveyorMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void conveyorStop(){
    topConveyorMotor.set(ControlMode.PercentOutput, 0.0);
    bottomConveyorMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setStopper(boolean state){
    stopper.set(state);
  }

  public void setIntake(boolean state){
    intakeSol.set(state);
  }

  public void setIntakeMotors(double speed){
    rightIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public int getConveyorEncoderDistance(){
    int distance = conveyorEncoder.getRaw();
    return distance;
  }

  public void resetEncoder(){
    conveyorEncoder.reset();
  }

  public boolean getSensorInput(){
    boolean feedback = infraredSensor.get();
    return feedback;
  }

  public boolean checker(){
    if(getSensorInput() == true){
      timer.reset();
    }
    if(timer.get() > 0.5){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
