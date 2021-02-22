/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * @author Daniel Pearson
 * @version 2/15/2020
 */
public class Intake extends SubsystemBase {
  private final DoubleSolenoid pistons;
  private final VictorSPX m_intake;

  public Intake() {
    pistons = new DoubleSolenoid(Constants.LEFTPISTON_A, Constants.LEFTPISTON_B);

    m_intake = new VictorSPX(Constants.INTAKE_MOTOR);
    m_intake.clearStickyFaults();

    m_intake.configFactoryDefault();

    m_intake.setInverted(InvertType.None);
    m_intake.setNeutralMode(NeutralMode.Brake);
  }

  public synchronized void extendPistons(){
    pistons.set(Value.kForward);
  }

  public synchronized void retractPistons(){
    pistons.set(Value.kReverse);
  }

  public synchronized void deployIntake(){
    extendPistons();
    m_intake.set(ControlMode.PercentOutput, 0.22);
  }

  public synchronized void retractIntake(){
    retractPistons();
    m_intake.set(ControlMode.PercentOutput, 0.0);
  }

  public synchronized void off(){
    pistons.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
