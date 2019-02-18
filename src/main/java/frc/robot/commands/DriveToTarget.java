/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveToTarget extends Command {

  PIDController turnController, distanceController;

  double distance;
  double angle;

  public DriveToTarget(double distance, double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distance = distance;
    this.angle = angle;
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    PIDSource angleSource = new PIDSource(){
    
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        
      }
    
      @Override
      public double pidGet() {
        return Robot.m_driveTrain.getAngle();
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
      }
    };

    PIDSource distanceSource = new PIDSource(){
    
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        
      }
    
      @Override
      public double pidGet() {
        return Robot.m_driveTrain.getAverageDistance();
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
      }
    };

    PIDOutput angleOut = new PIDOutput(){
    
      double output;

      public double get() {
        return output;
      }
      @Override
      public void pidWrite(double output) {
        this.output = output;
      }
    };

    PIDOutput distanceOut = new PIDOutput(){
    
      double output;
      public double get() {
        return output;
      }
      @Override
      public void pidWrite(double output) {
        this.output = output;
      }
    };

    

    turnController = new PIDController(1, 0, 0, angleSource, angleOut);
    turnController.setAbsoluteTolerance(0.1);
    turnController.setOutputRange(-0.2, 0.2);

    turnController.enable();

    distanceController = new PIDController(1, 0, 0, distanceSource, distanceOut);
    distanceController.setAbsoluteTolerance(1);
    distanceController.setOutputRange(-0.8, 0.8);
    distanceController.enable();
    
    
    
    

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double left = distanceController.get() + turnController.get();
    double right = distanceController.get() - turnController.get();
    Robot.m_driveTrain.drive(left,right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return turnController.onTarget() && distanceController.onTarget() ;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    distanceController.disable();
    turnController.disable();
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
