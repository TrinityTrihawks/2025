package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum SmartDashboardConstants {
    STAGED_PITCH_VELOCITY("Staged Pitch Velocity", 12),
    
    WRIST_TURN_OUTPUT("Wrist Turn Output", 1),
    WRIST_TURN_END_TARGET("Wrist Turn End Target", 0.5),
    WRIST_TURN_START_TARGET("Wrist Turn Start Target", 0),
    
    GRIPPER_INTAKE_VELOCITY("Gripper Intake Velocity", 5),
    
    CLIMB_VELOCITY("Climb Velocity", 6),
    
    TELESCOPE_VELOCITY("Telescope Velocity", 7),
    
    TELESCOPE_TARGET_MAX_OUTPUT("Telescope Target Max Output", 3),
    TELESCOPE_TARGET_POSITION_RETRACT("Telescope Target Position Retract", 0),
    TELESCOPE_TARGET_POSITION_L1("Telescope Target Position L1", 16),
    TELESCOPE_TARGET_POSITION_L2("Telescope Target Position L2", 65),
    TELESCOPE_TARGET_POSITION_L3("Telescope Target Position L3", 45);

    private final String name;
    private final double defaultValue;

    SmartDashboardConstants(String name, double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
    }

    public void setSmartDashboardDefault() {
        SmartDashboard.putNumber(name, defaultValue);
    }

    public double getSmartDashboardValue() {
        return SmartDashboard.getNumber(name, defaultValue);
    }
}
