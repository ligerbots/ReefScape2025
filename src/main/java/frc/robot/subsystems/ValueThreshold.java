package frc.robot.subsystems;

public class ValueThreshold {
    public enum Direction {RISING, FALLING};

    private final Direction m_direction;
    private final double m_threshold;
    private boolean m_previous = false;

    public ValueThreshold(Direction dir, double threshold) {
        m_direction = dir;
        m_threshold = threshold;
    }

    public boolean compute(double value) {
        boolean trigger = value > m_threshold;

        boolean result;
        if (m_direction == Direction.RISING) {
            result = trigger && !m_previous;
        } else {
            result = !trigger && m_previous;
        }
        
        m_previous = trigger;
        
        return result;
    }
}
