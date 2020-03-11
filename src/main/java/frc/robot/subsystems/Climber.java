/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class Climber extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DoubleSolenoid climbSol; 
  WPI_VictorSPX leftClimbMotor; 
  WPI_VictorSPX rightClimbMotor;
  SpeedControllerGroup climbMotors;
  DutyCycleEncoder rightClimbDCEncoder;
  DutyCycleEncoder leftClimbDCEncoder;
  Encoder rightClimbSEncoder;
  Encoder leftClimbSEncoder;

  public Climber(){
    climbSol = new DoubleSolenoid(Constants.forwardChannel, Constants.reverseChannel);
    leftClimbMotor = new WPI_VictorSPX(Constants.leftClimbMotor);
   // rightClimbMotor = new WPI_VictorSPX(Constants.rightClimbMotor);
    //leftClimbMotor.setInverted(true);
    //climbMotors = new SpeedControllerGroup(leftClimbMotor, rightClimbMotor);
    /*leftClimbDCEncoder = new DutyCycleEncoder(Constants.leftClimbDutyCycleEncoder);
    rightClimbDCEncoder = new DutyCycleEncoder(Constants.rightClimbDutyCycleEncoder);
    leftClimbSEncoder = new Encoder(Constants.leftClimbSEncoderA, Constants.leftClimbSEncoderB);
    rightClimbSEncoder = new Encoder(Constants.rightClimbSEncoderA, Constants.rightClimbSEncoderB);*/
  }

  public void extend(){
    climbSol.set(Value.kReverse);
  }

  public void retract(){
    climbSol.set(Value.kForward);
  }

  public void neutralize(){
    climbSol.set(Value.kOff);
  }

  public void setClimbMotors(double speed){
    //climbMotors.set(speed);
    leftClimbMotor.set(ControlMode.PercentOutput, speed);
  }

  /*public double getSEncoderRaw(){
    double r = rightClimbSEncoder.getRaw();
    double l = -leftClimbSEncoder.getRaw();
    double raw = (r + l)/2;
    return raw;
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
