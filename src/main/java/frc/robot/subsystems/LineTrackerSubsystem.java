/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.royals.sensors.LineTracker;

/**
 * Add your docs here.
 */
public class LineTrackerSubsystem extends Subsystem  {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public LineTracker lt;

  public LineTrackerSubsystem(){
          super("Line Tracker Subsystem");
          
          lt = new LineTracker(0,1,2);
          SmartDashboard.putData("Line Tracker", lt);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  


}
