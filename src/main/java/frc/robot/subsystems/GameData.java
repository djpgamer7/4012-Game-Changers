/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author Daniel Pearson
 * @version 2/18/2020
 */
public class GameData extends SubsystemBase {
  public GameData() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getGameData();
  }

  public synchronized void getGameData(){
    char data = 'n';


    if(DriverStation.getInstance().getGameSpecificMessage().length() > 0){
      data = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    }
    String toDashboard = Character.toString(data);

    SmartDashboard.putString("Control Color", toDashboard);
  }
}