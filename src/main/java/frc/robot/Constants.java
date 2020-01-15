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
    public static int climberSol1 = 0;
    public static int climberSol2 = 1;
    public static int climberSol3 = 2;
    public static int climberSol4 = 3;

    //Front Intake
    public static int intakeSol = 0;
    public static int leftIntakeMotor = 0;
    public static int rightIntakeMotor = 1;
}
