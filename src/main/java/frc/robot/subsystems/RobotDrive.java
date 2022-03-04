// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.commands.*;

public class RobotDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static CANSparkMax m_rearLeft = new CANSparkMax(AutoConstants.rearLeftDrive, MotorType.kBrushless);
  private static CANSparkMax m_frontLeft = new CANSparkMax(AutoConstants.frontLeftDrive, MotorType.kBrushless);
  private MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private static CANSparkMax m_rearRight = new CANSparkMax(AutoConstants.rearRightDrive, MotorType.kBrushless);
  private static CANSparkMax m_frontRight = new CANSparkMax(AutoConstants.frontRightDrive, MotorType.kBrushless);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  public static RelativeEncoder m_RLencoder = m_rearLeft.getEncoder();
  public static RelativeEncoder m_FLencoder = m_frontLeft.getEncoder();
  public static RelativeEncoder m_RRencoder = m_rearRight.getEncoder();
  public static RelativeEncoder m_FRencoder = m_frontRight.getEncoder();

  public static AHRS gyro = new AHRS(SPI.Port.kMXP);
  private static double error, derivative, adjust;
  private static int integral, previousError, setpoint = 0;


  public RobotDrive() {}

  public void arcadeDriveSimple(double leftStickPos, double rightStickPos, double maxSpeed){

    // if(Math.abs(leftStickPos) < 0.05 && Math.abs(rightStickPos) < 0.05){
    //     double speed = 0.2;
    //     while(speed > 0){
    //       m_left.set(speed);
    //       m_left.set(speed);
    //       speed -= 0.02;
    //     }

    //   }
    double drivePower = -1*Math.pow(leftStickPos, 3);
    double leftDrive = drivePower + rightStickPos*0.5; //*0.5
    double rightDrive  = drivePower - rightStickPos*0.5;
    leftDrive = leftDrive*maxSpeed;
    rightDrive = rightDrive*maxSpeed;

    if(!ShootingRotateCommand.rotateStatus){
    m_left.set(leftDrive);
    m_right.set(-rightDrive);
    System.out.println("Drive distance: " + getDistanceStraight());
    //ShootingRotate.adjustX();
    }

    //TEST THIS LATER
    // while(Math.abs(m_left.get()-leftDrive) < 0.1){
    //   if(m_left.get() < leftDrive){
    //     m_left.set(m_left.get() + 0.1);
    //   } else if(m_left.get() > leftDrive){
    //     m_left.set(m_left.get() - 0.1);
    //   }
    //   double rightSpeed = -1*m_right.get();
    //   if(rightSpeed < rightDrive){
    //     m_right.set(-1*(rightSpeed + 0.1));
    //   } else if(rightSpeed > rightDrive){
    //     m_right.set(-1*(m_right.get() - 0.1));
    //   }
    // }
    // m_left.set(leftDrive);
    // m_right.set(-rightDrive);

    }


  public static double PID(){
    error = 0 - gyro.getAngle();
    integral += (error*.02);
    derivative = (error-previousError)/.02;
    adjust = AutoConstants.kP * error + AutoConstants.kI*integral + AutoConstants.kD*derivative;
    return adjust;
  }

  public static double getLength(double ticks){
    return 3* ticks/(42.0)*6.0*Math.PI;

  }

  public void resetEncoders(){
    (RobotDrive.m_RLencoder).setPosition(0);
    (RobotDrive.m_RRencoder).setPosition(0);
    (RobotDrive.m_FLencoder).setPosition(0);
    (RobotDrive.m_FRencoder).setPosition(0);
  }

  public static double getDistanceStraight(){
    // ticks - 42
    // gearbox 12.75:1
    // 6 in diameter - 12 pi circumference
    // in inches
    return getLength((Math.abs(m_FLencoder.getPosition()) + Math.abs(m_FRencoder.getPosition())+Math.abs(m_RLencoder.getPosition())+Math.abs(m_RRencoder.getPosition()))/4.0);
  }

  public double getAngle(double ticks){
    return getLength(ticks)/(24.819*Math.PI)*360.0;
  }

  public double getTurnAmount(){
    return getAngle((Math.abs(m_FRencoder.getPosition())+Math.abs(m_RRencoder.getPosition())+Math.abs(m_RLencoder.getPosition())+Math.abs(m_RRencoder.getPosition()))/4.0);
  }


  public double getSpeed(){
    return (m_FLencoder.getVelocity() + m_FRencoder.getVelocity()+m_RLencoder.getVelocity()+m_RRencoder.getVelocity())/4;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.arcadeDriveSimple(RobotContainer.getLeftStickX(), RobotContainer.getLeftStickY(), .5);
    //System.out.println("Distance " + RobotDrive.getDistanceStraight());
    getDistanceStraight();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
