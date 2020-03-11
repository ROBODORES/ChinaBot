/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public double tv;
  public double tx;
  public double ty;
  public double ta;
  public double thor;
  public double tvert;

  public Limelight() {
  }

  public void updateRaw() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
  }

  public double getTx(){
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return tx;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
