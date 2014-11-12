package edu.oregonstate.robotics;
// adapted from https://github.com/lnanek/GlassSensorTest
import java.util.List;

import edu.oregonstate.robotics.R;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.view.WindowManager.LayoutParams;
import android.widget.TextView;

import android.widget.Toast;

public class MainActivity extends Activity {
	
	private static final String LOG_TAG = "SensorTest";
	private TextView text;
	private TextView locationText;
	private LocationManager mLocationManager;
	
	RosSensorIO sensorIO;
	
	private static void removeBackgrounds(final View aView) {
		aView.setBackgroundDrawable(null);
		aView.setBackgroundColor(Color.TRANSPARENT);
		aView.setBackgroundResource(0);
		if (aView instanceof ViewGroup) {
			final ViewGroup group = (ViewGroup) aView;
			final int childCount = group.getChildCount();
			for(int i = 0; i < childCount; i++) {
				final View child = group.getChildAt(i);
				removeBackgrounds(child);
			}
		}
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().addFlags(LayoutParams.FLAG_KEEP_SCREEN_ON);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);

		setContentView(R.layout.activity_main);
		
		text = (TextView) findViewById(R.id.text);

//        text.setText(BuildConfig.VERSION_NAME + ": " + BuildConfig.VERSION_CODE);

		locationText = (TextView) findViewById(R.id.location);

		
		mLocationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
		List<String> providers = mLocationManager.getAllProviders();
		for(String provider : providers ) {
			Log.i(LOG_TAG, "Found provider: " + provider);
		}

		// get the address/port
        // we're expecting a Text QR code in the form "aaa.bbb.ccc.ddd:pppp"
        if(this.getIntent().hasExtra("ROS_ADDR")) { // as an extra
            String addr = this.getIntent().getStringExtra("ROS_ADDR");
            this.startSensorIO(addr);
        } else { // or by scanning a barcode
            scanCode();
            // do the rest of this on the intent callback
            // http://zxing.appspot.com/generator/
        }
	}


	@Override
	protected void onPause() {
		super.onPause();
//		mSensorManager.unregisterListener(this);
	}


	@Override
	protected void onResume() {
		super.onResume();
	}
	
	private void scanCode() {
		Intent intent = new Intent("com.google.zxing.client.android.SCAN");
		intent.putExtra("SCAN_MODE", "QR_CODE_MODE");
		startActivityForResult(intent, 0);
	}

    private boolean startSensorIO(String addr) {
        String[] parts = addr.split(":");
        if(parts.length == 2) {
            Toast.makeText(this, "Connecting to " + addr, Toast.LENGTH_SHORT);
            this.sensorIO = new RosSensorIO(
                    this,
                    (SensorManager) getSystemService(Context.SENSOR_SERVICE),
                    parts[0],
                    Integer.parseInt(parts[1])
            );
            Thread clientThread = new Thread(this.sensorIO);
            clientThread.start();
            text.setText("Connected to " + addr + "\nSD=" + this.sensorIO.sd);
            return true;
        } else {
            return false;
        }
    }

	@Override
	public void onActivityResult(int requestCode, int resultCode, Intent intent) {
		if (requestCode == 0) {
			if (resultCode == RESULT_OK) {
				String scannedIp = intent.getStringExtra("SCAN_RESULT");
				if(!startSensorIO(scannedIp)) {
					Toast.makeText(this, "Invalid Address Format. Shold be 'IP:Port'", Toast.LENGTH_LONG).show();
					scanCode();
				}
			}
		}
	}

	//TODO this really should go in RosSensorIO.java if possible
    public boolean onKeyDown(int keycode, KeyEvent event) {
        if (keycode == KeyEvent.KEYCODE_DPAD_CENTER) {
            this.sensorIO.sendTap();
            return true;
        }
        
        return false;
    }

	
}
