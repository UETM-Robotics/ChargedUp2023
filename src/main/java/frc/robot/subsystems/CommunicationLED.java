public class CommunicationLED extends Subsystem implements CustomSubsystem {
    // Define Variables and States
    private Status currentStatus;
    private final AddressableLED indicatorLED;
    private final AddressableLEDBuffer indicatorLEDBuffer;

    // Set Up Instance Structure
    private static final CommunicationLED instance = new CommunicationLED();
    public static CommunicationLED getInstance() { return this.instance; }

    private CommunicationLED() {
        CurrentStatus = Status.NEUTRAL;

        AddressableLED = new AddressableLED(Constants.getLEDIndicatorPWMPin);
        indicatorLEDBuffer = new AddressableLEDBuffer(Constants.getLEDIndicatorSize);

        indicatorLED.setLength(indicatorLEDBuffer.getLength());
        indicatorLED.setData(indicatorLEDBuffer);
        indicatorLED.start();
    }

    // Define Looper
    private final Loop mLoop = new Loop() {
        @Override
        public void onFirstStart(double timestamp) {}

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (CommunicationLED.this) {
                switch(currentStatus) {
                    case NEUTRAL:
                        SmartDashboard.putString("Currently Requesting", "Nothing");
                        setColor(Constants.getLEDIndicatorNeutralColor);
                        break;
                    
                    case CONE:
                        SmartDashboard.putString("Currently Requesting", "Cone");
                        setColor(Constants.getLEDIndicatorConeColor);
                        break;
                    
                    case CUBE:
                        SmartDashboard.putString("Currently Requesting", "Cube");
                        setColor(Constants.getLEDIndicatorCubeColor);
                        break;
                    
                    default:
                        SmartDashboard.putString("ERROR", "Invalid LED State");
                }
            }
        }

        @Override
        public void onStop(double timestamp) {}
    }

    // Define Functions and Methods
    public void setColor(int[] rgbValues) {
        for (int i = 0; i < indicatorLEDBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, rgbValues[0], rgbValues[1], rgbValues[2]);
        }
    }

    // Define Auto-Generated Methods
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    // Define Setters
    public void setStatus(Status newStatus) {
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