package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberCfg {
    public static final int CLIMBER_SOLENOID_CHAN = 0;
    
    public static final Solenoid CLIMBER_SOLENOID = new Solenoid(PneumaticsModuleType.REVPH, CLIMBER_SOLENOID_CHAN);
    
}
