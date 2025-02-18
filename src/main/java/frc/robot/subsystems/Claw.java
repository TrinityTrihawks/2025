// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase {
  private final SparkMax m_grip = new SparkMax(18, MotorType.kBrushless);
  private RelativeEncoder grip_encoder;
  
  private final SparkMax m_wrist = new SparkMax(17, MotorType.kBrushless);
  private RelativeEncoder wrist_encoder1;
  private RelativeEncoder wrist_encoder2;
  
  // Set default brake mode
  private SparkMaxConfig wristConfig = new SparkMaxConfig();
  
  public Claw() {
    grip_encoder = m_grip.getEncoder();
    wrist_encoder1 = m_wrist.getEncoder();
    wrist_encoder2 = m_wrist.getAlternateEncoder();
    wristConfig.idleMode(IdleMode.kBrake);
    m_wrist.configure(wristConfig, null, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

}

