package frc.robot.subsystems.Shooter;

import java.util.List;

public class ShooterLookup {
    public static final class LookupTablePoint{
        double m_velocity;
        double m_hoodAngle;
        double m_distance;
    
        public LookupTablePoint(double meters, double velocity, double hoodAngle){
            this.m_hoodAngle = Math.toRadians(hoodAngle);
            this.m_distance = meters;
            this.m_velocity = velocity;
        }
        public LookupTablePoint(LookupTablePoint point){
            this.m_hoodAngle = point.m_hoodAngle;
            this.m_distance = point.m_distance;
            this.m_velocity = point.m_velocity;
        }
    }

    public static final List<LookupTablePoint> LookupTable = List.of(
        new LookupTablePoint(0, 1500, 14.5),
        new LookupTablePoint(1.32, 1500, 15),
        new LookupTablePoint(2, 2100, 17),
        new LookupTablePoint(2.42, 2200, 20),
        new LookupTablePoint(2.72, 2200, 22),
        new LookupTablePoint(3, 2200, 26),
        new LookupTablePoint(3.9, 2300, 28),
        new LookupTablePoint(4.6, 2450, 31),
        new LookupTablePoint(9.6, 4000, 33)
    );

    public static LookupTablePoint Lookup(double distance)
    {
        LookupTablePoint interpolated = new LookupTablePoint(LookupTable.get(0));

        for (int i = 1; i < LookupTable.size(); i++)
        {
            LookupTablePoint greater = LookupTable.get(i);

            if (LookupTable.get(i).m_distance > distance)  {
                interpolated.m_velocity =  interpolated.m_velocity + (distance - interpolated.m_distance )* (greater.m_velocity - interpolated.m_velocity)/(greater.m_distance-interpolated.m_distance);
                interpolated.m_hoodAngle = interpolated.m_hoodAngle + (distance - interpolated.m_distance) * (greater.m_hoodAngle - interpolated.m_hoodAngle)/(greater.m_distance-interpolated.m_distance);
                interpolated.m_distance = distance;
                return interpolated;

            }
            else {

                interpolated = new LookupTablePoint(LookupTable.get(i));
            }
        }

        return interpolated;
    }
}
