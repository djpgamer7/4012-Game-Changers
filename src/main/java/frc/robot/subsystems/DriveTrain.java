package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX m_leftFront;
  private final WPI_TalonFX m_leftBack;
  private final WPI_TalonFX m_rightFront;
  private final WPI_TalonFX m_rightBack;

  private final DifferentialDrive myRobot;

  private final ADXRS450_Gyro gyro;


  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  private final double unitsPerRev = 2048;

  private final double encoderConstant = (1 / 8.45) * (1 / unitsPerRev);

  Pose2d pose;


  public DriveTrain() {
    //Initiates the motor controllers
    m_leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    m_leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    m_rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    m_rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);

    gyro = new ADXRS450_Gyro();

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

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    m_leftFront.configAllSettings(configs);
    m_rightFront.configAllSettings(configs);

    m_leftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    m_rightFront.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

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

    m_leftBack.set(ControlMode.Follower, m_leftFront.getDeviceID());
    m_rightBack.set(ControlMode.Follower, m_rightFront.getDeviceID());

    myRobot = new DifferentialDrive(m_leftFront, m_rightFront);
    myRobot.setDeadband(0);
    myRobot.setSafetyEnabled(false);

    nullEncoders();
    zeroHeading();


    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    kinematics = new DifferentialDriveKinematics(.5969);



  }

  @Override
  public void periodic() {
    /*
    pose = odometry.update(getHeading(),
      m_leftFront.getSensorCollection().getIntegratedSensorPosition(),
      m_rightFront.getSensorCollection().getIntegratedSensorPosition()
    );
    */
    //odometry.update(Rotation2d.fromDegrees(getHeading()), this.getLeftDistance(), this.getRightDistance());
    double heading = getHeading();
    double leftDist = getLeftDistance();
    double rightDist = getRightDistance();

    odometry.update(
            Rotation2d.fromDegrees(heading),
            leftDist,
            rightDist
    );


    SmartDashboard.putNumber("Left Pos", getLeftDistance());
    SmartDashboard.putNumber("Right Pos", getRightDistance());
    SmartDashboard.putNumber("Left Speeds", getSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speeds", getSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Heading", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());

  }


  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }


  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
            (getLeftRPM() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60),
            (getRightRPM() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60)
    );
  }

  public synchronized void resetOdometry(Pose2d pose) {
    nullEncoders();
    zeroHeading();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public double getLeftDistance() {
    return -m_leftFront.getSelectedSensorPosition(0) * (((Math.PI * Units.inchesToMeters(6)) / 2048) / 10.71);
    //return -m_leftFront.getSelectedSensorPosition(0) * encoderConstant;
  }

  public double getRightDistance() {
    return -m_rightFront.getSelectedSensorPosition(0) * (((Math.PI * Units.inchesToMeters(6)) / 2048) / 10.71);
    //return -m_rightFront.getSelectedSensorPosition(0) * encoderConstant;
  }

  public double getDist() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  //Units: RPM
  public double getLeftRPM() {
    return -m_leftFront.getSelectedSensorVelocity(0) / unitsPerRev * 10 * 60;
  }

  //Units: RPM
  public double getRightRPM() {
    return -m_rightFront.getSelectedSensorVelocity(0) / unitsPerRev * 10 * 60;
  }


  public synchronized void nullEncoders() {
    m_rightFront.getSensorCollection().setIntegratedSensorPosition(0, 10);
    m_leftFront.getSensorCollection().setIntegratedSensorPosition(0, 10);
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

  public synchronized void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftFront.setVoltage(-leftVolts);
    m_rightFront.setVoltage(-rightVolts);
    myRobot.feed();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (true ? -1.0 : 1.0);
  }

  public synchronized void zeroHeading() {
    gyro.reset();
  }
}
