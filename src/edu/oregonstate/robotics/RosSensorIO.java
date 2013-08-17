package edu.oregonstate.robotics;

import java.net.*;
import java.io.*;
import java.nio.ByteBuffer;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.hardware.Sensor;
import de.timroes.axmlrpc.*;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;


public class RosSensorIO implements Runnable {
	XmlRpcClient client;
	XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
	URL serverURL;
	static final String LOG_TAG = "RosSensorIO";
	Handler handler;
	
	RosSensorIO(String serverName, int port) {
		try {
			Log.i(LOG_TAG, "Connecting to " + serverName + ":" + port);
			this.serverURL = new URL("http", serverName, port, "/android");
			this.config.setServerURL(this.serverURL);
		} catch (MalformedURLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
		
	public void callInLooper(final XmlRpcClient innerClient, final String method, final Object... params) {
		this.handler.postAtFrontOfQueue(new Runnable() {
			public void run() {
				long start = System.currentTimeMillis();
				try {
//					innerClient.call(method, params);
					innerClient.execute(method, params);
				} catch (XmlRpcException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				Log.i(LOG_TAG, "Took " + (System.currentTimeMillis() - start));
			}
		});
	}
	
	public void sendAcceleration(float ax, float ay, float az) {
		this.callInLooper(this.client, "imu", ax, ay, az);
	}
	
	public void sendOrientation(float roll, float pitch, float yaw) {
		this.callInLooper(this.client, "orientation", (int)roll, (int)pitch, (int)yaw);
	}

	public void run() {
		Looper.prepare();
		handler = new Handler();
		this.client = new XmlRpcClient();
		this.client.setConfig(this.config);
		Looper.loop();
	}

}
