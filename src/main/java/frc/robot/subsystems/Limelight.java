/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public double tv;
  public double tx;
  public double ty;
  public double ta;
  public double thor;
  public double tvert;

  public double tau = 0.35;

  boolean firstReading = true;

  public Limelight() {
    reset();
  }
  
  public double returnError(){
    return tx/tvert;
  }

  public void reset() {
    tx = 0.0;
    ty = 0.0;
    ta = 0.0;
    thor = 0.0;
    tvert = 0.0;
    updateLimelight();
    firstReading = true;
  }

  public void updateLimelight(){
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (tv > 0.0) {
      if (!firstReading) {
        tx += tau * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) - tx);
        ty += tau * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0) - ty);
        ta += tau * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) - ta);
        thor += tau * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0) - thor);
        tvert += tau * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0) - tvert);
      } else {
        updateRaw();
        firstReading = false;
      }
    } else {
      updateRaw();
      firstReading = true;
    }

    /*System.out.println("TVERT: " + tvert + ", THOR: " + thor);
    double angle = Math.acos((thor/tvert)/2.0);
    System.out.println("Angle: " + Math.toDegrees(angle));*/
  }

  public void updateRaw() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
