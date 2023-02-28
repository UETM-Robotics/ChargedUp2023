public class CommunicationLEDAction implements Action {
    CommunicationLED communicationLED;
    CommunicationLED.Status status;

    Supplier<Boolean> mButtonGetterMethod;

    public SetCommunicationLEDAction(CommunicationLED.Status status, Supplier<Boolean> buttonGetterMethod) {
        communicationLED = CommunicationLED.getInstance();
        mButtonGetterMethod = buttonGetterMethod;
        this.status = status;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        communicationLED.setStatus(status.NEUTRAL);
    }

    @Override
    public void start() {
        communicationLED.setStatus(status.NEUTRAL);
    }
}