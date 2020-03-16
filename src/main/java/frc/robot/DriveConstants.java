/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class DriveConstants {

        public static int leftMotor = 0;
        public static int rightMotor = 2;
        public static int leftSlaveMotor = 1;
        public static int rightSlaveMotor = 3; 
        public static final double ksVolts = 0.298; //not inverted. Inverted = 3.3557
        public static final double kvVoltSecondsPerMeter = 0.0449 ; //not inverted. Inverted = 22.2717
        public static final double kaVoltSecondsSquaredPerMeter = 0.00551 ; //not inverted. Inverted = 181.4882
        public static final double kPDriveVel = 0.254; 
        public static final double TrackwidthMeters = 23.79; 
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);
        public static final int LeftMotor1Port = 2;
        public static final int LeftMotor2Port = 3;
        public static final int RightMotor1Port = 0;
        public static final int RightMotor2Port = 1;
        public static final boolean GyroReversed = false;
        public static final int[] LeftEncoderPorts = new int[]{0, 1};
        public static  final int[] RightEncoderPorts = new int[]{2, 3};
        public static final boolean LeftEncoderReversed = false;
        public static final boolean RightEncoderReversed = false;
        public static final double EncoderDistancePerPulse = 1024; 
        
            
       

        public static final double WheelDiameterMeters = 0.1524;
        public static final double LeftEncoderPulsesPerRev = 4096 ; //
        public static final double RightEncoderPulsesPerRev = 4096 ; 
        public static final double LeftSlaveEncoderPulsesPerRev = 4096;
        public static final double RightSlaveEncoderPulsesPerRev = 4096;  
        public static final double LeftMetersPerPulse = Math.PI * WheelDiameterMeters / LeftEncoderPulsesPerRev;
        public static final double RightMetersPerPulse = Math.PI * WheelDiameterMeters / RightEncoderPulsesPerRev;
        public static final double LeftSlaveMetersPerPulse = Math.PI * WheelDiameterMeters / LeftSlaveEncoderPulsesPerRev; 
        public static final double RightSlaveMetersPerPulse = Math.PI * WheelDiameterMeters / RightSlaveEncoderPulsesPerRev;
        
	



      }
     

