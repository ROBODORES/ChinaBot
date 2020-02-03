/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonFX leftMotor;
  WPI_TalonFX rightMotor;
  WPI_TalonFX leftSlaveMotor;
  WPI_TalonFX rightSlaveMotor;
  DifferentialDrive differentialDrive;

  public Drivetrain(){
    leftMotor = new WPI_TalonFX(Constants.leftMotor);
    rightMotor = new WPI_TalonFX(Constants.rightMotor);
    leftSlaveMotor = new WPI_TalonFX(Constants.leftSlaveMotor);
    rightSlaveMotor = new WPI_TalonFX(Constants.rightSlaveMotor);
    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

    leftSlaveMotor.follow(leftMotor);
    rightSlaveMotor.follow(rightMotor);
  }

  public void arcadeDrive(double throttleSpeed, double turnSpeed){
    differentialDrive.arcadeDrive(throttleSpeed, turnSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
