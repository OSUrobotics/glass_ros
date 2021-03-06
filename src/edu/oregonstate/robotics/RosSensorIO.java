package edu.oregonstate.robotics;

import java.net.*;
import java.io.*;
import java.nio.ByteBuffer;
import java.util.List;

import android.util.Log;
import android.content.Context;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.google.android.glass.eye.EyeGesture;
import com.google.android.glass.eye.EyeGestureManager;

public class RosSensorIO implements Runnable, SensorEventListener {
	Socket client;
	DataOutputStream outStream;
	String serverName;
	int port;
	private static final String LOG_TAG = "SensorTest";
	private SensorManager mSensorManager;
	
	private EyeGestureManager mEyeGestureManager;
	private WinkReceiver mEyeEventReceiver;
	private WinkReceiver.EyeEventListener mEyeEventListener;

	public static byte TYPE_WINK = 98;
	public static byte TYPE_TAP  = 99;

	private Context mContext;
	
	private Sensor mOrientation, mAcceleration, mIlluminance, mProximity, mGyro, mRotation;

	
	public RosSensorIO(Context context, SensorManager sensorManager, String serverName, int port) {
		this.serverName = serverName;
		this.port = port;
		this.mSensorManager = sensorManager;
		this.mContext = context;
		
		List<Sensor> sensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
		for(Sensor sensor : sensors) {
			Log.i(LOG_TAG, "Found sensor: " + sensor.getName());
		}
		
		mOrientation = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
		mAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
		mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		mRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
		mIlluminance = mSensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
		mProximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
		
		
		mEyeGestureManager = EyeGestureManager.from(mContext);
		
		mEyeEventListener = new WinkReceiver.EyeEventListener() {
			@Override
			public void onWink() {
//				RosSensorIO.this.sendWink();
			}

			@Override
			public void onDoubleBlink() {
				RosSensorIO.this.sendWink();
			}
		};

		
		mEyeEventReceiver = new WinkReceiver(mEyeEventListener);
		mEyeGestureManager.stopDetector(EyeGesture.DOUBLE_BLINK);
		mEyeGestureManager.stopDetector(EyeGesture.WINK);
		mEyeGestureManager.enableDetectorPersistently(EyeGesture.DOUBLE_BLINK, true);
		mEyeGestureManager.enableDetectorPersistently(EyeGesture.WINK, true);
		IntentFilter eyeFilter = new IntentFilter("com.google.glass.action.EYE_GESTURE");
		eyeFilter.setPriority(3000);
		mContext.registerReceiver(mEyeEventReceiver, eyeFilter);
	}
	
	private synchronized void sendPrefixedTriplet(int prefix, float a, float b, float c) {
		try {			
			// this can be deserialized in python with struct.unpack('>3f', data[1:])
			// the first byte is a char representing which sensor the data is from
			ByteBuffer bb = ByteBuffer.allocate(16);
			bb.putInt(prefix).putFloat(a).putFloat(b).putFloat(c);//.putInt(999).putInt(999);
			this.outStream.write(bb.array());
		} catch (IOException e) {
			Log.w("RosSensorIO", "IO Exception: trying to reconnect");
			try {
				this.client.close();
				this.run();
			} catch (IOException e1) {
				e1.printStackTrace();
			}
			
		} catch (Exception e) {
			@SuppressWarnings("unused")
			int q = 5;
		}
	}
	
	public void sendAcceleration(float ax, float ay, float az) {
		this.sendPrefixedTriplet(Sensor.TYPE_LINEAR_ACCELERATION, ax, ay, az);
	}
	public void sendGyro(float vx, float vy, float vz) {
		this.sendPrefixedTriplet(Sensor.TYPE_GYROSCOPE, vx, vy, vz);
	}
	public void sendRotation(float ax, float ay, float az) {
		this.sendPrefixedTriplet(Sensor.TYPE_ROTATION_VECTOR, ax, ay, az);
	}
	
	public void sendOrientation(float roll, float pitch, float yaw) {
//		this.sendPrefixedTriplet(Sensor.TYPE_ORIENTATION, -roll, pitch, -yaw+180);
		this.sendPrefixedTriplet(Sensor.TYPE_ORIENTATION, -roll, pitch+90, -yaw-70);

	}
	
	public void sendLight(float l) {
		this.sendPrefixedTriplet(Sensor.TYPE_LIGHT, l, 0, 0);
	}
	
	public void sendTap() {
		this.sendPrefixedTriplet(TYPE_TAP, 0, 0, 0);
	}

	public void sendWink() {
		this.sendPrefixedTriplet(TYPE_WINK, 0, 0, 0);
	}
	
	
	public void run() {
		try {
			this.client = new Socket(this.serverName, this.port);
			OutputStream outToServer = client.getOutputStream();
			outStream = new DataOutputStream(outToServer);
		} catch (UnknownHostException e) {
			Log.e("RosSensorIO", "UnknownHostException", e);
		} catch (IOException e) {
			Log.e("RosSensorIO", "IOException", e);
		}
		mSensorManager.registerListener(this, mOrientation,
				SensorManager.SENSOR_DELAY_NORMAL);
		//mSensorManager.registerListener(this, mIlluminance,
		//		SensorManager.SENSOR_DELAY_NORMAL);
		//mSensorManager.registerListener(this, mProximity,
		//		SensorManager.SENSOR_DELAY_NORMAL);
//		mSensorManager.registerListener(this, mAcceleration,
//				SensorManager.SENSOR_DELAY_NORMAL);
//		mSensorManager.registerListener(this, mGyro,
//				SensorManager.SENSOR_DELAY_NORMAL);
//		mSensorManager.registerListener(this, mRotation,
//				SensorManager.SENSOR_DELAY_NORMAL);
	}

	
	@Override
	public void onSensorChanged(SensorEvent event) {
		if(Sensor.TYPE_ORIENTATION == event.sensor.getType()) {
			float azimuth_angle = event.values[0];
			float pitch_angle = event.values[1];
			float roll_angle = event.values[2];
			// Do something with these orientation angles.
			this.sendOrientation(roll_angle, pitch_angle, azimuth_angle);
		} else if(Sensor.TYPE_LINEAR_ACCELERATION == event.sensor.getType()) {
			float az = event.values[0];
			float ay = event.values[1];
			float ax = event.values[2];

			this.sendAcceleration(ax, ay, az);
		} else if(Sensor.TYPE_GYROSCOPE == event.sensor.getType()) {
			this.sendGyro(event.values[0], event.values[1], event.values[2]);
		} else if(Sensor.TYPE_ROTATION_VECTOR == event.sensor.getType()) {
			this.sendRotation(event.values[0], event.values[1], event.values[2]);
		} else if(Sensor.TYPE_LIGHT == event.sensor.getType()) {
			this.sendLight(event.values[0]);
		} else if(Sensor.TYPE_PROXIMITY == event.sensor.getType()) {
//			Log.i(LOG_TAG, "Got proximity: " + event.values[0]);
		}
	}
	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// Do something here if sensor accuracy changes.
		// You must implement this callback in your code.
	}
	
}
