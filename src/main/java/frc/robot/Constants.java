/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    public static int reverseChannel = 1;
    public static int leftClimbMotor = 4;
    public static int rightClimbMotor = 1;
    /*public static int leftClimbDutyCycleEncoder = 0;
    public static int rightClimbDutyCycleEncoder = 1;
    public static int leftClimbSEncoderA = 0;
    public static int leftClimbSEncoderB = 1;
    public static int rightClimbSEncoderA = 2;
    public static int rightClimbSEncoderB = 3;*/

    //Conveyor
    public static int bottomMotor = 2;
    public static int topMotor = 1;
    public static int stopper = 2;
    public static int intakeSol = 3;
    public static int leftIntakeMotor = 3;
    public static int rightIntakeMotor = 0;
    public static int conveyorEncoderA = 1;
    public static int conveyorEncoderB = 2;

    //LEDs
    public static int blinkinPort = 0;
}
