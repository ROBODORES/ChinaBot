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


}

    //Trajectories 
    // public static final double kTrackwidthMeters =
    //public static final DifferentialDriveKinematics kDriveKinematics =
            //new DifferentialDriveKinematics(kTrackwidthMeters);
            
       

       // public static final double WheelDiameterMeters = 0.;
       // public static final double LeftEncoderPulsesPerRev = ; //
        //public static final double RightEncoderPulsesPerRev = ; 
        //public static final double LeftMetersPerPulse = Math.PI * kWheelDiameterMeters / kLeftEncoderPulsesPerRev;
        //public static final double RightMetersPerPulse = Math.PI * kWheelDiameterMeters / kRightEncoderPulsesPerRev;

       // public static final boolean kGyroReversed = true; // For NavX

        /*public static final double ksVolts = 0.;
        public static final double kvVoltSecondsPerMeter = ;
        public static final double kaVoltSecondsSquaredPerMeter = ;
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel =; 
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7; }
    }
}
    */


