// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private static CANSparkMax m_Elevator = new CANSparkMax(AutoConstants.elevator, MotorType.kBrushless);
  private CANSparkMax m_Elevator2 = new CANSparkMax(AutoConstants.index2, MotorType.kBrushed);
  private CANSparkMax m_Elevator3 = new CANSparkMax(AutoConstants.index3, MotorType.kBrushless); //
  public static RelativeEncoder m_E1encoder = m_Elevator.getEncoder();

  public Elevator() {}
  //RR- methods 1/11/22
  public void moveUp(){
    m_Elevator.set(.5);
  }

  public void moveUp2(){
    m_Elevator2.set(-.5); 
  }

  public void moveUp3(){
    m_Elevator3.set(-.7);
  }


  public void moveDown(){
    m_Elevator.set(-.5);
  }

  public void moveDown2(){
    m_Elevator2.set(.5); 
  }

  public void moveDown3(){
    m_Elevator3.set(.7);
  }
  


  public void stop1(){
    m_Elevator.set(0);
  }

  public void stop2(){
    m_Elevator2.set(0);
  }

  public void stop3(){
    m_Elevator3.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // if (RobotContainer.m_joystick.getRawButton(1)) {
    //   moveUp();
    // } else {
    //   stop1();
    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
