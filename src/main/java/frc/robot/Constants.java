/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * @author Daniel Pearson
 * @version 2/24/2020
 */

@SuppressWarnings("unused")
public final class Constants {

    public static class AutoConstants {
        //NEED TO RE-TUNE ALL VALS
        public static final double ksVolts = .469;
        public static final double kvVoltSecondsPerMeter = .469;
        public static final double kaVoltSecondsSquaredPerMeter = .0263;

        public static final double kPDriveVel = .989;

        public static final double kMaxSpeed = 1;
        public static final double kMaxAcceleration = 1;

        public static final double kRameseteB = 2;
        public static final double kRameseteZeta = .7;
        public static final double trackWidthMeters = .7239;

        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(trackWidthMeters);
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
    public static final int SHOOTER_MOTOR1 = 5;
    public static final int SHOOTER_MOTOR2 = 10;
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
    public static final double shooterkP = .0001;
    public static final double shooterkI = .01;
    public static final double shooterkD = .001;

    //-----------------------------Shooter Constants-----------------------------//
    public static final double shooterSetPoint = 300;
}
