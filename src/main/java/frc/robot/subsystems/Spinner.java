/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class Spinner extends SubsystemBase {
  WPI_VictorSPX spinMotor;
  Solenoid spinSolenoid;
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;
  I2C.Port i2C = I2C.Port.kOnboard;
  Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    spinSolenoid = new Solenoid(Constants.spinSolPort);
    spinMotor = new WPI_VictorSPX(Constants.spinMotor);
    colorSensor = new ColorSensorV3(i2C);
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
  }

  public void spinTime(double spinSpeed) {
    spinMotor.set(spinSpeed);
  }

  public void setSpinnerSolenoid(boolean state){
    spinSolenoid.set(state);
  }

  public Color getCurrentColor(){
    Color sensedColor = colorSensor.getColor();
    return sensedColor;
  }

  public Color getBlue(){
    return kBlueTarget;
  }

  public Color getGreen(){
    return kGreenTarget;
  }

  public Color getRed(){
    return kRedTarget;
  }

  public Color getYellow(){
    return kYellowTarget;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
