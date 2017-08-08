package pl.edu.pw.meil.seekurJrRC;

import android.os.Bundle;
import android.preference.PreferenceActivity;

public class SeekurJrPreferenceActivity extends PreferenceActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            addPreferencesFromResource(R.xml.app_preferences);
            // Get the custom preference
    }
}
