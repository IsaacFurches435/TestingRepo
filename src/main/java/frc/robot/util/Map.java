package frc.robot.util;

public class Map {

    
    private double old_min;
    private double old_max;
    private double new_min;
    private double new_max;

    /**
     * Converts one range to a new range (eg, [-1, 1] --> [-100, 100])
     * @param old_min The minimum value prior to the new minimum value
     * @param old_max The maximum value prior to the new maximum value
     * @param new_min The new minimum value 
     * @param new_max The new maximum value;
     */
    public Map(double old_min, double old_max, double new_min, double new_max) {
        
        this.old_min = old_min;
        this.old_max = old_max;
        this.new_min = new_min;
        this.new_max = new_max; 
    }

    /**
     * Returns a new value based on the range specifed in the constructor
     * @param val value to convert to
     * @return a double that is the converted number to the range
     */
    public double getNewValue(double val) {
        return ((val - (old_min))) / (old_max - (old_min)) * (new_max - (new_min)) + (new_min);
    }
}
