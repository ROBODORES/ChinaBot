/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSensor extends SubsystemBase {
  /**
   * Creates a new UltrasonicSensor.
   */
  AnalogInput ultraSonicSensor;
  private double mmPerBit = 5;
  private double voltOffset = 2.5;
  private double bitsPerVolt = 409.6;
  private double mmToM = 1000;
  public UltrasonicSensor() {
    ultraSonicSensor = new AnalogInput(0);
  }

  public double getVolts(){
    return ultraSonicSensor.getVoltage();
  }

  /*public double getDistance(){
    double v = getVolts();
    double dInMeters = (v / bitsPerVolt);
    return dInMeters;
  }*/

  public void readSensorInput(){
    System.out.print("Voltage: " + getVolts());
    //System.out.println(", Distance in Meters: " + getDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
