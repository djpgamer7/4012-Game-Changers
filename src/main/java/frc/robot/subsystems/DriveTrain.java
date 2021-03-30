/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import com.analog.adis16470.frc.ADIS16470_IMU;

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

  private final DifferentialDrive myRobot;

  private final ADIS16470_IMU gyro;

  private final TalonFXSensorCollection leftEnc;
  private final TalonFXSensorCollection rightEnc;

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  private final double k_encoderConstant = (1 / 1) * (1 / 8192);

  //DiffDriveKinematics must be in SI units, inches to meters, track width!
  //public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  //public final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  //public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

  //public final PIDController leftPidController = new PIDController(AutoConstants.kP, 0, 0);
  //public final PIDController rightPidController = new PIDController(AutoConstants.kP, 0, 0);

  //Pose2d pose;

  public DriveTrain() {
    //Initiates the motor controllers
    m_leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    m_leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    m_rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    m_rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);

    gyro = new ADIS16470_IMU();

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

    //Sets the motors to brake mode
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);

    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    rightEnc = m_rightFront.getSensorCollection();
    leftEnc = m_leftFront.getSensorCollection();
        
    myRobot = new DifferentialDrive(m_leftFront, m_rightFront);

    myRobot.setDeadband(0);

    kinematics = new DifferentialDriveKinematics(.7239);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    myRobot.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    /*
    pose = odometry.update(getHeading(),
      m_leftFront.getSensorCollection().getIntegratedSensorPosition(),
      m_rightFront.getSensorCollection().getIntegratedSensorPosition()
    );
    */

    odometry.update(gyro.getRotation2d(), this.getLeftDistance(), this.getRightDistance());

    SmartDashboard.putNumber("Left Pos", getLeftDistance());
    SmartDashboard.putNumber("Right Pos", getRightDistance());
    SmartDashboard.putNumber("Velocity", getLeftVelocity());

    if(DriverStation.getInstance().isDisabled()) {
      resetSensors(getPose());
    }

  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
            this.getLeftVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60,
            this.getRightVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60
    );
  }

  public void resetSensors(Pose2d pose) {
    nullEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public double getLeftDistance() {
    return m_leftFront.getSelectedSensorPosition() * k_encoderConstant;
  }

  public double getRightDistance() {
    return m_rightFront.getSelectedSensorPosition() / k_encoderConstant;
  }

  public double getLeftVelocity() {
    return m_leftFront.getSelectedSensorVelocity() * k_encoderConstant * 10;
  }

  public double getRightVelocity() {
    return m_rightFront.getSelectedSensorVelocity() * k_encoderConstant * 10;
  }

  public TalonFXSensorCollection getLeftEnc() {
    return leftEnc;
  }

  public TalonFXSensorCollection getRightEnc() {
    return rightEnc;
  }

  public void nullEncoders() {
    leftEnc.setIntegratedSensorPosition(0, 10);
    rightEnc.setIntegratedSensorPosition(0, 10);
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  //Allows manual control of the robot

  /**
  *@param leftVal takes the value of the Y axis on the left joystick
  *@param rightVal takes the value of the Y axis on the left Joystick
  */
  public synchronized void manualDrive(double leftVal, double rightVal) {
    m_leftFront.set(ControlMode.PercentOutput, leftVal);
    m_rightFront.set(ControlMode.PercentOutput, rightVal);
  }

  /**
   * @param xVal takes through an xspeed 
   * @param yVal takes through a yspeed
   */
  public synchronized void arcadeDrive(double xVal, double yVal) {
    myRobot.arcadeDrive(xVal, yVal);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftFront.setVoltage(leftVolts);
    m_rightFront.setVoltage(-rightVolts);
    myRobot.feed();
  }


  public void setMaxOutput(double maxOutput) {
    myRobot.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }
}
