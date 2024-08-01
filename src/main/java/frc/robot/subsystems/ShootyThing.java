package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class ShootyThing extends SubsystemBase{

    private static ShootyThing instance = null;

    private final TalonFX m_topShooter = new TalonFX(1241341324); // these ids are placeholders
    private final TalonFX m_bottomShooter = new TalonFX(69420); // please dont actually use these ids

    private final CANSparkFlex m_indexMotor = new CANSparkFlex(3006, CANSparkFlex.MotorType.kBrushless); // rock red robot

    private final TimeOfFlight m_TOFSensor = new TimeOfFlight(9);

    private SmartDashboardNumber shootVel = new SmartDashboardNumber("shooter-shoot velocity", 0.5);
    private SmartDashboardNumber indexShootVel = new SmartDashboardNumber("shooter-index shoot velocity", 0.3);
    private SmartDashboardNumber ampTopVel = new SmartDashboardNumber("shooter-amp top velocity", 0.1);
    private SmartDashboardNumber ampBottomVel = new SmartDashboardNumber("shooter-amp bottom velocity", 0.2);
    private SmartDashboardNumber intakeVel = new SmartDashboardNumber("intake-intake velocity", -0.2);
    private SmartDashboardNumber intakeIndexVel = new SmartDashboardNumber("intake-index velocity", -0.2);

    private SmartDashboardNumber clearIndexThreshold = new SmartDashboardNumber("intake-clear index threshold", 300);
    private SmartDashboardNumber hasNoteThresholdDeviation = new SmartDashboardNumber("intake-note deviation threshold", 100);
                                

    private ShootyThing() {
        super("Shooter");

        this.m_topShooter.setInverted(false);
        this.m_topShooter.setNeutralMode(NeutralModeValue.Brake);

        this.m_bottomShooter.setInverted(true);
        this.m_bottomShooter.setNeutralMode(NeutralModeValue.Brake);

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(false);
        this.m_indexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        this.m_TOFSensor.setRangeOfInterest(8, 8, 12, 12);
        this.m_TOFSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    public void setShooterSpeed(double speed) {
        this.setShooterSpeed(speed, speed);
    }

    //TODO: implement motion magic
    public void setShooterSpeed(double top, double bottom){
        this.m_topShooter.set(top);
        this.m_bottomShooter.set(bottom);
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

    public static ShootyThing getInstance(){
        if (instance == null) instance = new ShootyThing();
        return instance;
    }
}