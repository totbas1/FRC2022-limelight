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
import edu.wpi.first.wpilibj.command.*;

import frc.robot.subsystems.RobotDrive;

public class PIDshootingRotate extends SubsystemBase implements PIDOutput{

  private static CANSparkMax m_Rotator;
  private static CANSparkMax m_angleRotator;
  public static RelativeEncoder m_AngleRotateEncoder;
  //public static DigitalInput leftRotateTurretLimitSwitch;
  //public static DigitalInput rightRotateTurretLimitSwitch;
  public final PIDController turretController;

  public final double kP = 0;
  public final double kI = 0;
  public final double kD = 0;
  public final double kF = 0;

  public PIDshootingRotate() {
    m_Rotator = new CANSparkMax(AutoConstants.shootRotate, MotorType.kBrushless);
    m_angleRotator = new CANSparkMax(AutoConstants.shootAngleRotate, MotorType.kBrushless);
    m_AngleRotateEncoder = m_angleRotator.getEncoder();
    //leftRotateTurretLimitSwitch = new DigitalInput(AutoConstants.leftRotateTurretLimitSwitch);
    //rightRotateTurretLimitSwitch = new DigitalInput(AutoConstants.rightRotateTurretLimitSwitch);
    turretController = new PIDController(Kp, Ki, Kd, source, output);

    //the turret goes from -110 to 75 degrees, but since the sensor we're using is an encoder this 
    //value should be converted to rotations. for now I will leave it at degrees.
    turretController.setInputRange(-108.0f, 73.0f);
    //speed of the motor. .35 is reasonable
    turretController.setOutputRange(-0.35, 0.35);
    //2 degrees of tolerance
    turretController.setAbsoluteTolerance(2.0f);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   resetHood();
   runXMotor(RobotContainer.getRightStickXAxis());
  }

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
    while(ShootingRotate.m_AngleRotateEncoder.getPosition() < (desiredAngleToRotations(desiredAngleY) - 30))
    {
      m_angleRotator.set(speedY);
    }
  }

  public void resetHood(){
    final double speedY = 0.35;
    while(RobotContainer.hoodLimitSwitch.get() == true){
      System.out.println("fj");
      m_angleRotator.set(-1*speedY);
    }
    m_angleRotator.set(0);
    (ShootingRotate.m_AngleRotateEncoder).setPosition(0);

  }

  public double desiredAngleToRotations(double desiredAngleY){
    //with (1/1000 * 34/100.8) gear ratio -> .1214 degrees per rotation
    double rotations = desiredAngleY/.1214;
    return rotations;
  }

  public void runXMotor(double rightStickXaxis){
    double speed = 0.4;
    double motorDrive = rightStickXaxis*speed;
    m_Rotator.set(motorDrive);
  }

  public void limeLightAutoAdjustX(double offsetX){
    while(Math.abs(offsetX) > 4 ){
      final double speedX = 0.4;
      if(offsetX > 0 ){
        m_Rotator.set(-1*speedX);
      }
      if(offsetX < 0){
        m_Rotator.set(1*speedX);
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

  public void rotateDegrees(double angle){
    (ShootingRotate.m_AngleRotateEncoder).setPosition(0);
    turretController.reset();
    turretController.setPid(kP, kI, kD);
    turretController.setSetpoint(angle);
    turretController.enable();
  }

  @Override
  public void pidWrite(double output){
    set(ControlMode.PercentOutput, output);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
