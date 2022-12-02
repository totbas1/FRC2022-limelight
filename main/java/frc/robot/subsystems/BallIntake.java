// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;

public class BallIntake extends SubsystemBase {
  /** Creates a new BallIntake. */
  private CANSparkMax m_Intake = new CANSparkMax(AutoConstants.intakePort, MotorType.kBrushless);

  public BallIntake() {}

  public void ballTake(){//intake- RR
    m_Intake.set(.34);
  }

  public void ballTakeReverse(){//move wheels out- RR
    m_Intake.set(-.7);
  }
  
  public void stop(){//stop- RR
    m_Intake.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotContainer.leftTriggerAxis()){
      this.ballTake();
    } else{
      this.stop();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
