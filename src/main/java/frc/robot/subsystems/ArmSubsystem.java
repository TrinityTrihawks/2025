// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmSubsystem extends SubsystemBase {
  
  private final SparkMax m_arm = new SparkMax(15, MotorType.kBrushless);
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;

  private final SparkMax m_grip = new SparkMax(16, MotorType.kBrushless);
  private RelativeEncoder grip_encoder;
  
  private final SparkMax m_wrist = new SparkMax(17, MotorType.kBrushless);
  private RelativeEncoder wrist_encoder1;
  private RelativeEncoder wrist_encoder2;

  private final SparkMaxConfig configarm = new SparkMaxConfig();

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(OperatorConstants.kMaxVelocity, OperatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(OperatorConstants.kP, OperatorConstants.kI, OperatorConstants.kD, m_constraints, OperatorConstants.kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(OperatorConstants.kS, OperatorConstants.kG, OperatorConstants.kV);

  /**
   * 
   */
  public ArmSubsystem() {
    encoder1 = m_arm.getEncoder();            // motor encoder
    encoder2 = m_arm.getAlternateEncoder();   // secondary encoder
    grip_encoder = m_grip.getEncoder();
    wrist_encoder1 = m_wrist.getEncoder();
    wrist_encoder2 = m_wrist.getAlternateEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("motor speed", get_speed());
    SmartDashboard.putNumber("motor position", get_Position());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

  public void setMotorSpeed(double speed) {
    m_arm.set(speed); // speed is a value between -1 and 1
  }

  public double get_speed(){
    return encoder1.getVelocity();
  }

  public double get_Position(){
    return encoder2.getPosition();
  }

  public double grip_vel(){
    return grip_encoder.getVelocity();
  }

  public void grip(double grip_speed) {
    m_grip.setVoltage(grip_speed);
  }
  
  public double get_wrist_pos1(){
    return wrist_encoder1.getPosition();
  }

  public double get_wrist_pos2(){
    return wrist_encoder2.getPosition();
  }

  public void turn_wrist(double wrist_speed) {
    m_wrist.setVoltage(wrist_speed);
  }

  public void turn_wrist_pos(double speed, double position) {
    m_wrist.setVoltage(speed);
  }

  public void wrist_stop(){
    m_wrist.setVoltage(0);
  }

  public void set_goal(double goal){
    m_controller.setGoal(goal);
    SmartDashboard.putNumber("goal", goal);
  }

  public void run_goal(){
    m_wrist.setVoltage(
        m_controller.calculate(get_wrist_pos1())
            + m_feedforward.calculate(m_controller.getSetpoint().velocity));      
    SmartDashboard.putNumber("current", get_wrist_pos1());
  }
}
