/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
//import com.analog.adis16470.frc.ADIS16470_IMU;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * @author Daniel Pearson
 * @version 2/21/2020
 */
public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX m_leftFront;
  private final WPI_TalonFX m_leftBack;
  private final WPI_TalonFX m_rightFront;
  private final WPI_TalonFX m_rightBack;

  //private final DifferentialDrive myRobot;

  //private final ADIS16470_IMU gyro;

  //DiffDriveKinematics must be in SI units, inches to meters, track width!
  public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  //public final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

  public final PIDController leftPidController = new PIDController(AutoConstants.kP, 0, 0);
  public final PIDController rightPidController = new PIDController(AutoConstants.kP, 0, 0);

  Pose2d pose;

  public DriveTrain() {
    //Initiates the motor controllers
    m_leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    m_leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    m_rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    m_rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);

    //gyro = new ADIS16470_IMU();

    //Clears the motor controllers faults
    m_leftFront.clearStickyFaults();
    m_leftBack.clearStickyFaults();
    m_rightFront.clearStickyFaults();
    m_rightBack.clearStickyFaults();

    m_leftFront.configFactoryDefault();
    m_leftBack.configFactoryDefault();
    m_rightFront.configFactoryDefault();
    m_rightBack.configFactoryDefault();

    //Sets the Back motors to follow the front
    m_leftBack.set(ControlMode.Follower, m_leftFront.getDeviceID());
    m_rightBack.set(ControlMode.Follower, m_rightFront.getDeviceID());

    //Determines which motors will be inverted
    m_leftFront.setInverted(InvertType.InvertMotorOutput);
    m_leftBack.setInverted(InvertType.FollowMaster);
    m_rightFront.setInverted(InvertType.None);
    m_rightBack.setInverted(InvertType.FollowMaster);

    //Sets the motos to brake mode
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);
        
    //myRobot = new DifferentialDrive(m_leftFront, m_rightFront);
  }

  @Override
  public void periodic() {
    /*
    pose = odometry.update(getHeading(), 
      m_leftFront.getSensorCollection().getIntegratedSensorPosition(), 
      m_rightFront.getSensorCollection().getIntegratedSensorPosition()
    );
    */ 
  }

  /*
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      m_leftFront.getSensorCollection().getIntegratedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60,
      m_rightFront.getSensorCollection().getIntegratedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60
    );
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedForward;
  }

  public Pose2d getPose() {
    return pose;
  }

  public PIDController getLeftPIDController(){
    return leftPidController;
  }

  public PIDController getRightPIDController(){
    return rightPidController;
  }
  */

  //Allows manual control of the robot
  /**
  *@param leftVal takes the value of the Y axis on the left joystick
  *@param rightVal takes the value of the Y axis on the left Joystick
  */
  public synchronized void manualDrive(double leftVal, double rightVal) {
    m_leftFront.set(ControlMode.PercentOutput, leftVal);
    m_rightFront.set(ControlMode.PercentOutput, rightVal);
  }
  /*
  public synchronized void voltageDrive(double leftVolts, double rightVolts){
    m_leftFront.setVoltage(leftVolts);
    m_rightFront.setVoltage(-rightVolts);
    myRobot.feed();
  }*/

  /**
   * @param xVal takes through an xspeed 
   * @param yVal takes through a yspeed
   */
  public synchronized void arcadeDrive(double xVal, double yVal) {
   // myRobot.arcadeDrive(xVal, yVal);
  }
}
