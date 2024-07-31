package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooteyThing extends SubsystemBase{

    private static ShooteyThing instance = null;

    private final TalonFX m_topShooter = new TalonFX(1241341324); // these ids are placeholders
    private final TalonFX m_bottomShooter = new TalonFX(69420); // please dont actually use these ids
    private final CANSparkFlex m_indexMotor = new CANSparkFlex(3006, CANSparkFlex.MotorType.kBrushless); // rock red robot

    private final double kIntakeSpeed = -0.1;
    private final double kShootSpeed = 0.1; // slow ahh shooter

    private final double kNoteSpike = -69; // tune this
    private final double kStartSpike = 69; // tune this

    private ShooteyThing() {
        super("ShooteyThing");

        this.m_topShooter.setInverted(false);
        this.m_topShooter.setNeutralMode(NeutralModeValue.Brake);

        this.m_bottomShooter.setInverted(true);
        this.m_bottomShooter.setNeutralMode(NeutralModeValue.Brake);

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(false);
        this.m_indexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public void setShooterSpeed(double speed) {
        this.m_topShooter.set(speed);
        this.m_bottomShooter.set(speed);
    }

    public static ShooteyThing getInstance(){
        if (instance == null) instance = new ShooteyThing();
        return instance;
    }
}