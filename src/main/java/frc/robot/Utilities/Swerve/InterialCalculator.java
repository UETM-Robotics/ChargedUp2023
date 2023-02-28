package frc.robot.Utilities.Swerve;

public class InterialCalculator {

    private double currentInertia = 0;

    private double degradationRate = 0;
    private double incrementRate = 0;
    private double inertialThreshold = 0;

    private double inertialMaximum = 0;

    /**
     * 
     * @param degredation_rate the rate at which the inertia will degrade
     * 
     * @param increment_rate the rate at which the inertia will increment
     * 
     * @param inertial_threshold the threshold at which the inertia will no longer attempt to change
     * 
     */
    public InterialCalculator(double degredation_rate, double increment_rate, double inertial_threshold, double inertial_maximum) {

        degradationRate = degredation_rate;
        incrementRate = increment_rate;

        inertialThreshold = inertial_threshold;

        inertialMaximum = inertial_maximum;

    }

    public double update(double updateRate) {

        if( Math.abs(updateRate) > 0.001 ) {

            currentInertia += Math.abs( Math.abs(currentInertia) - inertialMaximum) > inertialThreshold ? updateRate * incrementRate : 0;

        } else {

            currentInertia += currentInertia > 0 ? -degradationRate : degradationRate;

        }

        return currentInertia;

    }
    
}