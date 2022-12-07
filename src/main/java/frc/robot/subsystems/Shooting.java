// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimeLight;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */
  private WPI_TalonFX m_ShooterTopRight = new WPI_TalonFX(AutoConstants.shooter_right);
  //private WPI_TalonFX m_ShooterTopLeft = new WPI_TalonFX(AutoConstants.shooter_left);
  private double error, derivative, adjust;
  private int integral, previousError = 0;
  private double setpoint = 0.0;
  private static boolean constantMode = true;
  private static boolean adjustMode = true;
  private static double speed = 0;

  public Shooting() {}
  //methods written by RR on 1/11/22
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Constant shoot? ", constantMode);
    SmartDashboard.putBoolean("Adjust mode? ", adjustMode);
    SmartDashboard.putNumber("Speed Error", RobotContainer.distanceError);
    SmartDashboard.putNumber("Speed", speed);
    //double speed = 0;
    if(RobotContainer.m_driverController2.getBButtonPressed()){
      switchMode();
    }
    if(RobotContainer.m_driverController2.getYButtonPressed()){
      switchMode2();
    }
    if(constantMode && (RobotContainer.rightTriggerAxis()||RobotContainer.m_driverController.getBButtonPressed())){
      speed = 0.565 + RobotContainer.distanceError;
      this.shootTop(speed);
    }
    else if(!constantMode && (RobotContainer.rightTriggerAxis()||RobotContainer.m_driverController.getBButtonPressed())){
      speed = LimeLight.calcSpeed() + RobotContainer.distanceError;
      this.shootTop(speed);
    } else{
      this.stop();
    }


  }

  
  public void shootTop(double strength) {
    m_ShooterTopRight.set(strength);
    double tS  = strength;
    double current = 0;
    while(current < tS){
      current += 0.005;
      m_ShooterTopRight.set(current);
    }
        
  }

  public static boolean constantStatus(){
   // return constantMode;
   return false;
  }
  public static boolean adjustStatus(){
    //return adjustMode;
    return false;
  }
  public void switchMode(){
   // constantMode = !constantMode;
  }

  public void switchMode2(){
   // adjustMode = !adjustMode;
  }

  /*public double PID(double q){
    setpoint = q;
    error = setpoint - m_ShooterTop.getSelectedSensorVelocity();
    integral += (error*.02);
    derivative = (error-this.previousError)/.02;
    adjust = AutoConstants.kP * error + AutoConstants.kI*integral + AutoConstants.kD*derivative;
    return adjust;
  }*/

  public void stop() {
    m_ShooterTopRight.set(0);
    //m_ShooterTopLeft.set(0);*/


  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
