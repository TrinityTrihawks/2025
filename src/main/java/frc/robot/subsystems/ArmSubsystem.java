package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem  extends SubsystemBase {
    
    SparkMax max = new SparkMax(15, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    RelativeEncoder encoder1 = max.getEncoder();
    RelativeEncoder encoder2 = max.getAlternateEncoder();

    public ArmSubsystem() {}

    public double pos1(){
        return encoder1.getPosition();
    }

    public double pos2(){
        return encoder2.getPosition();
    }

    public double vel1(){
        return encoder1.getVelocity();
    }

    public double vel2(){
        return encoder2.getVelocity();
    }
    
    public Command spin(double speed){
        return run(() -> max.setVoltage(speed));
    }

    @Override
    public void periodic() {

    }
}
