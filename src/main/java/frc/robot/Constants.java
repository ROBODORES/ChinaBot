/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Drivetrian
     public static int leftMotor = 0;
    public static int rightMotor = 2;
    public static int leftSlaveMotor = 1;
    public static int rightSlaveMotor = 3;
    
    //Climber
    public static int forwardChannel = 0;


    //encoder ports

        public static final int[] LeftEncoderPorts = new int[]{0, 1};
        public static final int[] RightEncoderPorts = new int[]{2, 3};
        public static final boolean LeftEncoderReversed = true;
        public static final boolean RightEncoderReversed = false;
    
    //Front Intake
    public static int intakeSol = 0;
    public static int leftIntakeMotor = 0;
    public static int rightIntakeMotor = 1;

    //Conveyor
    public static int bottomMotor = 0;
    public static int topMotor = 1;
    public static int stopper = 0;

    //LEDs
    public static int blinkinPort = 0; 
    

    //encoder values  
    public static int leftClimbDutyCycleEncoder = 0;
    public static int rightClimbDutyCycleEncoder = 1;
    public static int leftClimbSEncoderA = 0;
    public static int leftClimbSEncoderB = 1;
    public static int rightClimbSEncoderA = 2;
    public static int rightClimbSEncoderB = 3;




    //Trajectories 
   public static final double TrackwidthMeters = 23.79; 
   public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TrackwidthMeters);
            
       

        public static final double WheelDiameterMeters = 0.1524;
        public static final double LeftEncoderPulsesPerRev = 4096 ; //
        public static final double RightEncoderPulsesPerRev = 4096 ; 
        public static final double LeftSlaveEncoderPulsesPerRev = 4096;
        public static final double RightSlaveEncoderPulsesPerRev = 4096;  
        public static final double LeftMetersPerPulse = Math.PI * WheelDiameterMeters / LeftEncoderPulsesPerRev;
        public static final double RightMetersPerPulse = Math.PI * WheelDiameterMeters / RightEncoderPulsesPerRev;
        public static final double LeftSlaveMetersPerPulse = Math.PI * WheelDiameterMeters / LeftSlaveEncoderPulsesPerRev; 
        public static final double RightSlaveMetersPerPulse = Math.PI * WheelDiameterMeters / RightSlaveEncoderPulsesPerRev; 
        public static final double MaxSpeedMetersPerSecond = 0.2; 
        public static final double MaxAccelerationMetersPerSecondSquared = 3; 

        public static final boolean GyroReversed = true;

        public static final double ksVolts = 0.298; //not inverted. Inverted = 3.3557
        public static final double kvVoltSecondsPerMeter = 0.0449 ; //not inverted. Inverted = 22.2717
        public static final double kaVoltSecondsSquaredPerMeter = 0.00551 ; //not inverted. Inverted = 181.4882
        public static final double kPDriveVel = 0.0004; // with unit as inches = 0.254
    
        //  baseline values for a RAMSETE follower in units m/s
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7; }
    

    


