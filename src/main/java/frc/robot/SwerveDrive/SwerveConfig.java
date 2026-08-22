package frc.robot.SwerveDrive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

public class SwerveConfig {
    public final XboxController xboxController =new XboxController(0);
    // =========================
    // Pigeon2.0
    // =========================
    public final Pigeon2 pigeon2 = new Pigeon2(13);
    // =========================
    // Pigeon2.0
    // =========================

    // =========================
    // CAN IDs - COLOQUE OS SEUS
    // =========================

    public final CANcoder canFL = new CANcoder(10);
    public final CANcoder canFR = new CANcoder(11);
    public final CANcoder canBL = new CANcoder(9);
    public final CANcoder canBR = new CANcoder(12);

    // =========================
    // MOTORES DE ANGLE
    // =========================

    // 3 Minions = TalonFXS
    public final TalonFXS motorBR = new TalonFXS(8);
    public final TalonFXS motorFR = new TalonFXS(6);
    public final TalonFXS motorBL = new TalonFXS(2);
    public final TalonFXS motorIndex1 = new TalonFXS(19);

    // Kraken X44
    public final TalonFX motorFL = new TalonFX(4);

    //MOTORS DRIVE 
    public final TalonFX drive_motorBR = new TalonFX(7);
    public final TalonFX drive_motorFR = new TalonFX(5);
    public final TalonFX drive_motorBL = new TalonFX(1);
    public final TalonFX drive_motorFL = new TalonFX(3);
    //MOTORS DRIVE
    // OFFSETS DOS CANCODERS
    public final double offsetFL = 277.29492187;
    public final double offsetFR = 289.07226562;
    public final double offsetBL = 276.06445312;
    public final double offsetBR = 93.8671875;

    // =========================
    // CONTROLES
    // =========================

    public final DutyCycleOut outputFL = new DutyCycleOut(0);
    public final DutyCycleOut outputFR = new DutyCycleOut(0);
    public final DutyCycleOut outputBL = new DutyCycleOut(0);
    public final DutyCycleOut outputBR = new DutyCycleOut(0);

    // kP inicial
    public final PIDController pidFL = new PIDController(0.01, 0, 0);
    public final PIDController pidFR = new PIDController(0.01, 0, 0);
    public final PIDController pidBL = new PIDController(0.01, 0, 0);
    public final PIDController pidBR = new PIDController(0.01, 0, 0);

}
