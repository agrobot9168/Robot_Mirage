  package frc.robot;
   import com.ctre.phoenix6.hardware.TalonFXS;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
   import edu.wpi.first.wpilibj2.command.SubsystemBase;

  public class Garra extends SubsystemBase {
     private final GarraConfig garraConfig = new GarraConfig();
     
@Override
  public void periodic() {
     Double corrente = garraConfig.getCurrent();
      
       SmartDashboard.putNumber("CORRENTE", corrente);
       }
    
  }
