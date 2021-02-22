/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Daniel Pearson
 * @version 2/15/2020
 */
public class Pneumatics extends SubsystemBase {
  private Compressor comp;

  public Pneumatics() {
    comp = new Compressor();
    comp.clearAllPCMStickyFaults();
  }

  public void oxygenCapture(){
    comp.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
