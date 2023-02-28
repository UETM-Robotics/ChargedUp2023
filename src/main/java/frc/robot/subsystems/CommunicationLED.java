public class CommunicationLED extends Subsystem implements CustomSubsystem {
    // Define Variables and States
    private Status currentStatus;

    private final int redPin;
    private final int greenPin;
    private final int bluePin;

    // Set Up Instance Structure
    private static final CommunicationLED instance = new CommunicationLED;
    public static CommunicationLED getInstance() { return this.instance; }

    private CommunicationLED() {
        currentStatus = Status.NEUTRAL;

        redPin = Constants.getRedLEDPin();
        greenPin = Constants.getGreenLEDPin();
        bluePin = Constants.getBlueLEDPin();
    }

    // Define Looper
    private final Loop mLooper = new Loop() {
        @Override
        public void onFirstStart(double timestamp) {}

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (CommunicationLED.this) {
                switch(currentStatus) {
                    case NEUTRAL:
                        SmartDashboard.putString("Currently Requesting", "Nothing")
                        break;
                    
                    case CONE:
                        SmartDashboard.putString("Currently Requesting", "Cone")
                        break;
                    
                    case CUBE:
                        SmartDashboard.putString("Currently Requesting", "Cube")
                        break;
                    
                    default:
                        SmartDashboard.putString("ERROR", "Invalid LED State")
                }
            }
        }

        @Override
        public void onStop(double timestamp) {}
    }

    // Define Setters
    public void setState(Status newStatus) {
        if (currentStatus != newStatus) {
            currentStatus = newStatus;
        }
    }

    // Define Enums
    public enum Status {
        NEUTRAL,
        CONE,
        CUBE
    }
}