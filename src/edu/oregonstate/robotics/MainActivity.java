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

		locationText = (TextView) findViewById(R.id.location);

		
		mLocationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
		List<String> providers = mLocationManager.getAllProviders();
		for(String provider : providers ) {
			Log.i(LOG_TAG, "Found provider: " + provider);
		}
		
		// scan a barcode
		scanCode();
		// do the rest of this on the intent callback
		// we're expecting a Text QR code in the form "aaa.bbb.ccc.ddd:pppp"
		// http://zxing.appspot.com/generator/
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
	
	@Override
	public void onActivityResult(int requestCode, int resultCode, Intent intent) {
		if (requestCode == 0) {
			if (resultCode == RESULT_OK) {
				String scannedIp = intent.getStringExtra("SCAN_RESULT");
				String[] parts = scannedIp.split(":");
				if(parts.length == 2) {
					this.sensorIO = new RosSensorIO((SensorManager) getSystemService(Context.SENSOR_SERVICE), parts[0], Integer.parseInt(parts[1]));
							Thread clientThread = new Thread(this.sensorIO);
							clientThread.start();
				} else {
					Toast.makeText(this, "Invalid Address Format. Shold be 'IP:Port'", Toast.LENGTH_LONG).show();
					scanCode();
				}
			}
		}
	}
}
