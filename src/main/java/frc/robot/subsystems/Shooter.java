/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 * @author Daniel Pearson 
 * @version 3/2/2020
 */
public class Shooter extends SubsystemBase {
  public static CANSparkMax shooterMotor1;
 // public static CANSparkMax shooterMotor2;
  public static CANEncoder shooterEnc;

  public Shooter() {
    shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR1, CANSparkMax.MotorType.kBrushless);
    //shooterMotor2 = new CANSparkMax(Constants.SHOOTER_MOTOR2, CANSparkMax.MotorType.kBrushless);

    shooterMotor1.clearFaults();
    shooterMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //shooterMotor2.clearFaults();
    //shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    shooterMotor1.setInverted(true);
    //shooterMotor2.setInverted(true);


    shooterEnc = shooterMotor1.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getRpm());
    SmartDashboard.putNumber("Shooter Pos", getPosition());
  }

  public double getRpm() {
    return -shooterEnc.getVelocity();
  }

  public double getPosition() {
    return shooterEnc.getPosition();
  }

  public synchronized void setVelocity(double velocity) {
    double shooterVal = 0;
    if(getRpm() < velocity) {
      double diff = getRpm() - velocity;

      shooterVal = diff * Constants.shooterkP;
      if(shooterVal > .7) {
        shooterVal = .7;
      }
    } else if(getRpm() > velocity) {
      double diff = velocity - shooterEnc.getVelocity();

      shooterVal = diff * Constants.shooterkP;

      if(shooterVal < -.7) {
        shooterVal = -.7;
      }
    }



    shooterMotor1.set(shooterVal);
  }

  public synchronized void setSpeed(double speed) {
    shooterMotor1.set(speed);
    //shooterMotor2.set(speed);
  }

  public boolean atSetpoint() {
    if(this.getRpm() == Constants.shooterSetPoint) {
      return true;
    }
    return false;
  }

  public void stop() {
    shooterMotor1.set(0);
    //shooterMotor2.set(0);
  }
  /*

  public double getRpm(){
    return m_shooter.getSensorCollection().getIntegratedSensorVelocity();
  }

  public synchronized void activateClosedLoopControl(){
    m_shooter.set(ControlMode.Velocity, targetVelocityPer100ms);
  }

  public synchronized void setVelocity(double velocity){
    double setVelocity = velocity * 500 * 4096 / 600;
    m_shooter.set(ControlMode.Velocity, setVelocity);
  }

  public synchronized void setPercent(double percent){
    m_shooter.set(ControlMode.PercentOutput, percent);
  }
  
  public boolean atSetpoint(){
    if(this.getRpm() == Constants.shooterSetPoint){
      return true;
    }
    return false;
<<<<<<< HEAD
  }*/
}

