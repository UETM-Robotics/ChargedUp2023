public class CommunicationLEDAction implements Action {
    CommunicationLED communicationLED;
    CommunicationLED.Status status = CommunicationLED.Status.NEUTRAL;

    Supplier<Boolean> mButtonGetterMethod;

    public CommunicationLEDAction() {
        communicationLED = CommunicationLED.getInstance();
        communicationLED.updateStatus(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {}
}