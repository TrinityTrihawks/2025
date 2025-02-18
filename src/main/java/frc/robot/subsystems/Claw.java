// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase {
  private final SparkMax m_grip = new SparkMax(18, MotorType.kBrushless);
  private RelativeEncoder grip_encoder;
  
  private final SparkMax m_wrist = new SparkMax(17, MotorType.kBrushless);
  private RelativeEncoder wrist_encoder1;
<<<<<<< HEAD
  private RelativeEncoder wrist_encoder2;
=======
  private AlternateEncoder wrist_encoder2;
>>>>>>> 2c6e436526f85542b799e7f74569062fafa98f4c
  
  /** Creates a new ExampleSubsystem. */
  public Claw() {
    grip_encoder = m_grip.getEncoder();

    wrist_encoder1 = m_wrist.getEncoder();
    wrist_encoder2 = m_wrist.getAlternateEncoder();
  }


  
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public double grip_vel(){
    return grip_encoder.getVelocity();
  }

  public void grip(double grip_speed) {
    m_grip.setVoltage(grip_speed);
  }
  
  public double get_wrist_encoder1(){
    return wrist_encoder1.getPosition();
  }

  public double get_wrist_encoder2(){
    return wrist_encoder2.getPosition();
  }

  public void turn_wrist(double wrist_speed) {
    m_wrist.setVoltage(wrist_speed);
  }

  public void turn_wrist_pos(double speed, double position) {
    m_wrist.setVoltage(speed);
  }
  

  
}

