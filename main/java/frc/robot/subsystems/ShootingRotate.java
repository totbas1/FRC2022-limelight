// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimeLight;

import frc.robot.subsystems.RobotDrive;

public class ShootingRotate extends SubsystemBase {
  /** Creates a new ShootingRotate. */
  private static CANSparkMax m_Rotator = new CANSparkMax(AutoConstants.shootRotate, MotorType.kBrushless);
  private static CANSparkMax m_angleRotator = new CANSparkMax(AutoConstants.shootAngleRotate, MotorType.kBrushless);

  public static RelativeEncoder m_AngleRotateEncoder = m_angleRotator.getEncoder();

  public double currentPos = 0.0;
  public double newPos = 0.0;

  public static DigitalInput leftRotateTurretLimitSwitch = new DigitalInput(AutoConstants.leftRotateTurretLimitSwitch);
  public static DigitalInput rightRotateTurretLimitSwitch = new DigitalInput(AutoConstants.rightRotateTurretLimitSwitch);

  public ShootingRotate() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   if(LimeLight.limelightTrackTarget()){
      System.out.println("Robot is ready to shoot");
    }
    else{
      System.out.println("Robot is aiming...");
      resetHood();
      System.out.println(LimeLight.limelightTrackingX());
      SmartDashboard.putNumber("LimelightX", LimeLight.limelightTrackingX());
      move(LimeLight.limelightTrackingX(), LimeLight.calcHoodAngle());
    }

    //SmartDashboard.putNumber("Hood Angle: ", getZEncoder());
    //SmartDashboard.putNumber("Distance", LimeLight.calcDistance(LimeLight.limelightTrackingY()));
    //move(RobotContainer.getJoystickXAxis(),RobotContainer.getJoystickYAxis());
  }

  //Methods written by RR 1/11/2022
  //trajectoryAngle is y-axis, aimAngle is x-axis
  public void move(double offsetX, double desiredAngleY){

    //no PID loop
    while(Math.abs(offsetX) > 2 ){
      final double speedX = 0.2;
      if(offsetX > 0 ){
        m_Rotator.set(-1*speedX);
      }
      if(offsetX < 0){
        m_Rotator.set(1*speedX);
      }
    }
    m_Rotator.set(0);

    //no PID loop 
    final double speedY = 0.35;
    resetHood();
    while(ShootingRotate.m_AngleRotateEncoder.getPosition() < (encoderToAngle.rotations - 30))
    {
      m_angleRotator.set(speedY);
    }
  }

  public void resetHood(){
    final double speedY = 0.35;
    while(RobotContainer.hoodLimitSwitch.get() == false){
      m_angleRotator.set(-1*speedY);
    }
  }

  public void desiredAngleToRotations(int desiredAngleY){
    //with (1/1000 * 34/100.8) gear ratio -> .1214 degrees per rotation
    int rotations = desiredAngleY/.1214;
  }

  public void runMotor(double speed){
   // m_angleRotator.set(speed);
  }

  public double getZEncoder(){
    //return m_AngleRotateEncoder.getPosition();
    return 0;
  }

  public void stop(){
   // m_Rotator.set(0);
  }

  public void adjustX(){
    while(Math.abs(LimeLight.limelightTrackingX()) > 3){
        final double speed = 0.3;
        if(LimeLight.limelightTrackingX() > 0 ){
          m_Rotator.set(-1*speed);
        }
        if(LimeLight.limelightTrackingX() < 0){
          m_Rotator.set(1*speed);

      }
    }
    m_Rotator.set(0);
  }

  /**
   *  method to adjust the hood to a given angle, assuming the relativee ecnoder is 0 at the limit switch.
   * @param angle - angle we want to adjust to
   */
  public void adjustHood(){
    //no PID loop
    double angle = LimeLight.calcHoodAngle();
    final double speed = 0.35;
    while (Math.abs(ShootingRotate.m_AngleRotateEncoder.getPosition()-angle) > 10){
      if(ShootingRotate.m_AngleRotateEncoder.getPosition() > angle){
        m_angleRotator.set(-1*speed);
      } else{
        m_angleRotator.set(speed);
      }
    }
    m_angleRotator.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
