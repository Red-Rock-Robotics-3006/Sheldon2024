package frc.robot.Utils3006;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardNumber {
    private double defaultValue;
    private double lastValue;
    private String key;

    public SmartDashboardNumber(String key, double defaultValue){
        this.key = key;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;

        SmartDashboard.putNumber(this.key, this.defaultValue);
    }

    public void putNumber(double val){
        SmartDashboard.putNumber(this.key, val);
    }

    public void setDefaultValue(double val){
        this.defaultValue = val;
    }

    public double getNumber(){
        this.lastValue = SmartDashboard.getNumber(this.key, this.defaultValue);

        return this.lastValue;
    }

    public boolean hasChanged(){
        boolean b = Double.compare(SmartDashboard.getNumber(key, defaultValue), lastValue) != 0;
        getNumber();
        return b;
    }
}
