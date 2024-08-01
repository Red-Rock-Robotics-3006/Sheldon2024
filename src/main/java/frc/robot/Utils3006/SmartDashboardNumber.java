package frc.robot.Utils3006;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardNumber {
    private double defaultValue;
    private String key;

    public SmartDashboardNumber(String key, double defaultValue){
        this.key = key;
        this.defaultValue = defaultValue;

        SmartDashboard.putNumber(this.key, this.defaultValue);
    }

    public void putNumber(double val){
        SmartDashboard.putNumber(this.key, val);
    }

    public void setDefaultValue(double val){
        this.defaultValue = val;
    }

    public double getNumber(){
        double m = SmartDashboard.getNumber(this.key, this.defaultValue);
        return m;
    }
}
