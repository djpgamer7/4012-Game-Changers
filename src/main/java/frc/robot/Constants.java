/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * @author Daniel Pearson
 * @version 2/24/2020
 */

@SuppressWarnings("unused")
public final class Constants {

    public class AutoConstants {
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kP = 9.95;
    }
    //-----------------------------Subsystems-----------------------------//
    //DriveBase
    public static final int LEFT_FRONT = 1; //d1
    public static final int LEFT_BACK = 2; //d2
    public static final int RIGHT_FRONT = 3; //d3
    public static final int RIGHT_BACK = 4; //d4
    //Intake
    public static final int INTAKE_MOTOR = 8; //Purple
    public static final int LEFTPISTON_A = 0; 
    public static final int LEFTPISTON_B = 1;
    //Hopper
    public static final int CONVEYOR_MOTOR = 6; //Grey
    public static final int FEEDER_MOTOR = 7; //Blue
    //Shooter
    public static final int SHOOTER_MOTOR = 5;  
    //Wheel Spinner
    public static final int WHEEL_MOTOR = 9; //White
    //LED's
    public static final int LEDSTRIP = 0;
    //-----------------------------OI-----------------------------//
    public static final int LEFT_JOY = 0;
    public static final int RIGHT_JOY = 1;
    public static final int MANIP_CONTROLLER = 2;
    //-----------------------------PID VALUE-----------------------------//
    //DO NOT RUN ANYTHING OFF OF THESE VALUES
    public static final double shooterkP = .1;
    public static final double shooterkI = .01;
    public static final double shooterkD = .001;

    //-----------------------------Shooter Constants-----------------------------//
    public static final double shooterSetPoint = 300;
}
