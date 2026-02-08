package frc.robot;

import java.util.function.Consumer;

import choreo.auto.AutoChooser;

/**
 * A version of Choreo's AutoChooser that allows registering a method to be 
 * called whenever the value changes.
 */
public class NotifyingAutoChooser extends AutoChooser {
    private Consumer<String> m_listener = null;
    private String lastKnownValue = "Unknown";

    public NotifyingAutoChooser() {
        super();
    }

    @Override 
    public String select(String selectStr) {
        String result = super.select(selectStr);

        if (m_listener != null && lastKnownValue != selectStr) {
            m_listener.accept(selectStr);
        }

        lastKnownValue = selectStr;

        return result;
    }

    /**
     * Bind a listener that's called when the selected value changes. Only one
     * listener can be bound.
     * Calling this function will replace the previous listener.
     *
     * @param listener The function to call that accepts the new value
     */
    public void onChange(Consumer<String> listener) {
        m_listener = listener;
    }

    /**
     * Return the last known selected menu item from this chooser.
     */
    public String lastKnownSelection() {
        return lastKnownValue;
    }

}
