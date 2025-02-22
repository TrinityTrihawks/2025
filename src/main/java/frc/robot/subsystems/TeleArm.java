// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TeleArm extends SubsystemBase {

   private final SparkMax m_tele = new SparkMax(19, MotorType.kBrushless);
  private RelativeEncoder tele_encoder1;
  /** Creates a nampleSubsystemew ExampleSubsystem. */
  public TeleArm() {
    tele_encoder1 = m_tele.getEncoder();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
   
  

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

  public double get_tele_encoder1(){
    return tele_encoder1.getPosition();
}
public void telescope(double telescope_speed) {
  m_tele.setVoltage(telescope_speed);
}
}
