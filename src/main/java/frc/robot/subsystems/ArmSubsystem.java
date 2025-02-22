// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax m_climb = new SparkMax(15, MotorType.kBrushless);
  private RelativeEncoder climb_encoder1;
  private AbsoluteEncoder climb_encoder2;
  private final SparkMax m_staged = new SparkMax(16, MotorType.kBrushless);
  private RelativeEncoder staged_encoder1;
  private AbsoluteEncoder staged_encoder2;
  
  
  private SparkMaxConfig wristConfig;
  

  private final SparkMaxConfig configarm = new SparkMaxConfig();
  
  private SparkClosedLoopController m_wrist_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public ArmSubsystem() {
    climb_encoder2 = m_climb.getAbsoluteEncoder();
    climb_encoder1 = m_climb.getEncoder();
    staged_encoder1 = m_staged.getEncoder();
    staged_encoder2 = m_staged.getAbsoluteEncoder();
    
    
    
   // m_wrist_pidController = m_wrist.getClosedLoopController();
    wristConfig = new SparkMaxConfig();
    wristConfig.encoder.positionConversionFactor(1);
    wristConfig.encoder.velocityConversionFactor(1);
   
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double get_pitch_encoder1(){
    return staged_encoder1.getPosition();
  }

  public double get_pitch_encoder2(){
    return staged_encoder2.getPosition();
  }
  public void staged_pitch(double staged_speed) {
    m_staged.setVoltage(staged_speed);
  }
  public double get_climb_encoder1(){
    return climb_encoder1.getPosition();
  }
  public double get_climb_encoder2(){
    return climb_encoder2.getPosition();
  }
  public void climb(double climb_speed) {
    m_climb.setVoltage(climb_speed);
  }
}
