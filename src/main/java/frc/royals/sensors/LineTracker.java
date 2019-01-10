/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.royals.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Line Tracker class for getting position of 3 Allen Bradley photo sensors
 */
public class LineTracker {
    private DigitalInput leftLineTracker, centerLineTracker, rightLineTracker;
    private int[] lines = new int[3];

    /**
     * Constructs a LineTracker object using 3 DIO ports connected to sensors
     * @param leftLineTracker port that left line sensor is connceted to 
     * @param centerLineTracker port that center line sensor is connected to
     * @param rightLineTracker port that right line sensor is connected to
     */
    public LineTracker(int leftLineTracker, int centerLineTracker, int rightLineTracker) {
        this.leftLineTracker = new DigitalInput(leftLineTracker);
        this.centerLineTracker = new DigitalInput(centerLineTracker);
        this.rightLineTracker = new DigitalInput(rightLineTracker);
        
    }

    /**
     * Will return a 3 digit int that represents the status of the line sensors
     * e.g. 100, 010, 001 more info to come
     * @return a 3digit integer reflecting the status of the sensors
     */
    public int getLinePostion() {
        lines[0] = leftLineTracker.get() ? 1 :0;
        lines[1] = centerLineTracker.get() ? 1 : 0;
        lines[2] = rightLineTracker.get() ? 1 : 0;
        return Integer.parseInt(""+lines[0] + lines[1] + lines[2]);
      }

      @Override
      public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("LineTracker");
          builder.addValueProperty(key, getter, setter);

      }
}
