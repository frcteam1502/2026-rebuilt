package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberCfg {
    public static final int CLIMBER_SOLENOID1_CHAN = 0;
    public static final int CLIMBER_SOLENOID2_CHAN = 1;
    
    public static final Solenoid CLIMBER_SOLENOID1 = new Solenoid(PneumaticsModuleType.REVPH, CLIMBER_SOLENOID1_CHAN);
    public static final Solenoid CLIMBER_SOLENOID2 = new Solenoid(PneumaticsModuleType.REVPH, CLIMBER_SOLENOID2_CHAN);
    
}
