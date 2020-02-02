/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;



/**
 * Add your docs here.
 */
public class PathFinder extends SubsystemBase {

	
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final String trajectoryJSON = "paths/YourPath.wpilib.json";{
    }{
    try {
      final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      final Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (final IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }
  final String trajectoryJSON = "paths/YourPath.wpilib.json";{
  
  }}
}}


