package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class ShooteyThing extends SubsystemBase{

    private static ShooteyThing instance = null;

    private final TalonFX m_topShooter = new TalonFX(1241341324); // these ids are placeholders
    private final TalonFX m_bottomShooter = new TalonFX(69420); // please dont actually use these ids

    private Slot0Configs topSlot0Configs, bottomSlot0Configs;

    private final CANSparkFlex m_indexMotor = new CANSparkFlex(3006, CANSparkFlex.MotorType.kBrushless); // rock red robot

    private final TimeOfFlight m_TOFSensor = new TimeOfFlight(9);

    private SmartDashboardNumber shootVel = new SmartDashboardNumber("shooter-shoot velocity", 3000);
    private SmartDashboardNumber indexShootVel = new SmartDashboardNumber("shooter-index shoot velocity", 1800);
    private SmartDashboardNumber ampTopVel = new SmartDashboardNumber("shooter-amp top velocity", 300);
    private SmartDashboardNumber ampBottomVel = new SmartDashboardNumber("shooter-amp bottom velocity", 1200);
    private SmartDashboardNumber intakeVel = new SmartDashboardNumber("intake-intake velocity", -1200);
    private SmartDashboardNumber intakeIndexVel = new SmartDashboardNumber("intake-index velocity", -1200);

    private SmartDashboardNumber clearIndexThreshold = new SmartDashboardNumber("intake-clear index threshold", 300);
    private SmartDashboardNumber hasNoteThresholdDeviation = new SmartDashboardNumber("intake-note deviation threshold", 100);

    private SmartDashboardNumber topShooterKs = new SmartDashboardNumber("shooter-top/kS", 0.0);
    private SmartDashboardNumber topShooterKa = new SmartDashboardNumber("shooter-top/kA", 0.0);
    private SmartDashboardNumber topShooterKv = new SmartDashboardNumber("shooter-top/kV", 0.132);
    private SmartDashboardNumber topShooterKp = new SmartDashboardNumber("shooter-top/kP", 0.4);
    private SmartDashboardNumber topShooterKi = new SmartDashboardNumber("shooter-top/kI", 0.0);
    private SmartDashboardNumber topShooterKd = new SmartDashboardNumber("shooter-top/kD", 0.0);
  
    private SmartDashboardNumber bottomShooterKs = new SmartDashboardNumber("shooter-bottom/kS", 0.0);
    private SmartDashboardNumber bottomShooterKa = new SmartDashboardNumber("shooter-bottom/kA", 0.0);
    private SmartDashboardNumber bottomShooterKv = new SmartDashboardNumber("shooter-bottom/kV", 0.133);
    private SmartDashboardNumber bottomShooterKp = new SmartDashboardNumber("shooter-bottom/kP", 0.4);
    private SmartDashboardNumber bottomShooterKi = new SmartDashboardNumber("shooter-bottom/kI", 0.0);
    private SmartDashboardNumber bottomShooterKd = new SmartDashboardNumber("shooter-bottom/kD", 0.0);
                                

    private ShooteyThing() {
        super("ShooteyThing");

        if (!Utils.isSimulation()) {
            this.m_topShooter.getConfigurator().apply(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withPeakForwardDutyCycle(1d)
                    .withPeakReverseDutyCycle(-1d)
                    .withNeutralMode(NeutralModeValue.Brake)
            );
            
            this.m_bottomShooter.getConfigurator().apply(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withPeakForwardDutyCycle(1d)
                    .withPeakReverseDutyCycle(-1d)
                    .withNeutralMode(NeutralModeValue.Brake)
            );
    
            this.topSlot0Configs = new Slot0Configs()
                .withKS(topShooterKs.getNumber())
                .withKA(topShooterKa.getNumber())
                .withKV(topShooterKv.getNumber())
                .withKP(topShooterKp.getNumber())
                .withKI(topShooterKi.getNumber())
                .withKD(topShooterKd.getNumber());
            
            this.bottomSlot0Configs = new Slot0Configs()
                .withKS(bottomShooterKs.getNumber())
                .withKA(bottomShooterKa.getNumber())
                .withKV(bottomShooterKv.getNumber())
                .withKP(bottomShooterKp.getNumber())
                .withKI(bottomShooterKi.getNumber())
                .withKD(bottomShooterKd.getNumber());
    
            this.m_topShooter.getConfigurator().apply(topSlot0Configs);
            this.m_bottomShooter.getConfigurator().apply(bottomSlot0Configs);

        }

        

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(false);
        this.m_indexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        this.m_TOFSensor.setRangeOfInterest(8, 8, 12, 12);
        this.m_TOFSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    public void setShooterSpeed(double speed) {
        this.setShooterSpeed(speed, speed);
    }

    /**
     * 
     * @param top top velocity in RPM
     * @param bottom bottom velocity in RPM
     */
    public void setShooterSpeed(double top, double bottom){
        this.m_topShooter.set(top);
        this.m_bottomShooter.set(bottom);

        if (Double.compare(top, 0d) == 0) {
            this.m_topShooter.setControl(new DutyCycleOut(0d).withOverrideBrakeDurNeutral(true));
        } else {
            this.m_topShooter.setControl(new VelocityVoltage(top / 60d)
                                            .withSlot(0)
                                            .withEnableFOC(true)
                                            .withOverrideBrakeDurNeutral(true));
        }

        if (Double.compare(top, 0d) == 0) {
            this.m_bottomShooter.setControl(new DutyCycleOut(0d).withOverrideBrakeDurNeutral(true));
        } else {
            this.m_bottomShooter.setControl(new VelocityVoltage(bottom / 60d)
                                            .withSlot(0)
                                            .withEnableFOC(true)
                                            .withOverrideBrakeDurNeutral(true));
        }
    }

    public void setShootSpeed(){
        this.setShooterSpeed(shootVel.getNumber());
    }

    public void setAmpSpeed(){
        this.setShooterSpeed(ampTopVel.getNumber(), ampBottomVel.getNumber());
    }

    public void shoot(){
        this.m_indexMotor.set(indexShootVel.getNumber());
    }

    public void intake(){
        this.setShooterSpeed(intakeVel.getNumber());
        this.m_indexMotor.set(intakeIndexVel.getNumber());
    }

    public boolean hasNote(){
        return this.m_TOFSensor.getRange() < this.clearIndexThreshold.getNumber() - this.hasNoteThresholdDeviation.getNumber();
    }

    //TODO: implement
    public boolean atTargetVelocity(){
        return true;
    }

    public void stop(){
        this.setShooterSpeed(0);
        this.m_indexMotor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("intake-has note", this.hasNote());
        SmartDashboard.putNumber("intake-TOF range", this.m_TOFSensor.getRange());

        //thanks bread <3
        if (topShooterKs.hasChanged()
            || topShooterKv.hasChanged()
            || topShooterKp.hasChanged()
            || topShooterKi.hasChanged()
            || topShooterKd.hasChanged()
            || topShooterKa.hasChanged()) {
            topSlot0Configs.kS = topShooterKs.getNumber();
            topSlot0Configs.kV = topShooterKv.getNumber();
            topSlot0Configs.kP = topShooterKp.getNumber();
            topSlot0Configs.kI = topShooterKi.getNumber();
            topSlot0Configs.kD = topShooterKd.getNumber();
            topSlot0Configs.kA = topShooterKa.getNumber();

            if (!Utils.isSimulation()) this.m_topShooter.getConfigurator().apply(bottomSlot0Configs);
            else System.out.println("applyied");
        }

        if (bottomShooterKs.hasChanged()
            || bottomShooterKv.hasChanged()
            || bottomShooterKp.hasChanged()
            || bottomShooterKi.hasChanged()
            || bottomShooterKd.hasChanged()
            || bottomShooterKa.hasChanged()) {
            bottomSlot0Configs.kS = bottomShooterKs.getNumber();
            bottomSlot0Configs.kV = bottomShooterKv.getNumber();
            bottomSlot0Configs.kP = bottomShooterKp.getNumber();
            bottomSlot0Configs.kI = bottomShooterKi.getNumber();
            bottomSlot0Configs.kD = bottomShooterKd.getNumber();
            bottomSlot0Configs.kA = bottomShooterKa.getNumber();

            if (!Utils.isSimulation()) this.m_bottomShooter.getConfigurator().apply(bottomSlot0Configs);
            else System.out.println("applied2");
        }
    }

    public Command shootCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> this.setShootSpeed(), 
                () -> {}, 
                (interrupted) -> {}, 
                () -> this.atTargetVelocity(), this),
            new FunctionalCommand(
                () -> this.shoot(), 
                () -> {}, 
                (interrupted) -> this.stop(), 
                () -> !this.hasNote(), this)
        );
    }

    public Command ampCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> this.setAmpSpeed(), 
                () -> {}, 
                (interrupted) -> {}, 
                () -> this.atTargetVelocity(), this),
            new FunctionalCommand(
                () -> this.shoot(), 
                () -> {}, 
                (interrupted) -> this.stop(), 
                () -> !this.hasNote(), this)
        );
    }

    public Command intakeCommand(){
        return new FunctionalCommand(
            () -> this.intake(), 
            () -> {}, 
            (interrupted) -> this.stop(), 
            () -> this.hasNote(), this);
    }

    public Command stopCommand(){
        return this.runOnce(() -> this.stop());
    }

    public static ShooteyThing getInstance(){
        if (instance == null) instance = new ShooteyThing();
        return instance;
    }
}