package edu.oregonstate.robotics;

import org.json.JSONException;
import org.json.JSONObject;

public class RosbridgeRequest extends JSONObject {
	public RosbridgeRequest(String op) throws JSONException {
		this.put("op", op);
	}
	
	public static RosbridgeRequest advertise(String topic, String type) throws JSONException {
		RosbridgeRequest req = new RosbridgeRequest("advertise");
		req.put("topic", topic);
		req.put("type", type);
		return req;
	}
	public static RosbridgeRequest advertise(String id, String topic, String type) throws JSONException {
		RosbridgeRequest req = advertise(topic, type);
		req.put("id", id);
		return req;
	}

	public static RosbridgeRequest publish(String topic, JSONObject msg) throws JSONException {
		RosbridgeRequest req = new RosbridgeRequest("publish");
		req.put("topic", topic);
		req.put("msg", msg);
		return req;
	}
	public static RosbridgeRequest publishEmpty(String topic) throws JSONException {
		JSONObject emptyObj = new JSONObject();
		return publish(topic, emptyObj);
	}
	
	
	public static JSONObject header(String frameId) throws JSONException {
		JSONObject msgHeader = new JSONObject();
		msgHeader.put("frame_id", frameId);
		msgHeader.put("stamp", System.currentTimeMillis());
		return msgHeader;
	}
	
	public static JSONObject vector3(double x, double y, double z) throws JSONException {
		JSONObject msgVec3 = new JSONObject();
		msgVec3.put("x", x);
		msgVec3.put("y", y);
		msgVec3.put("z", z);		
		return msgVec3;
	}
	
	public static JSONObject quaternion(double x, double y, double z, double w) throws JSONException {
		JSONObject quat = new JSONObject();
		quat.put("x", x);
		quat.put("y", y);
		quat.put("z", z);
		quat.put("w", w);		
		return quat;
	}
	
	public static RosbridgeRequest publishIMU(String topic, double linearX, double linearY, double linearZ) throws JSONException {	
		JSONObject msgImu = new JSONObject();
		msgImu.put("header", header("face_detection"));
		msgImu.put("linear_acceleration", vector3(linearX, linearY, linearZ));
		return publish(topic, msgImu);
	}
	
	public static JSONObject pose(JSONObject position, JSONObject orientation) throws JSONException {
		JSONObject pose = new JSONObject();
		JSONObject poseStamped = new JSONObject();

		pose.put("position", position);
		pose.put("orientation", orientation);
		
		poseStamped.put("header", header("face_detection"));
		poseStamped.put("pose", pose);
		
		return poseStamped;
	}
	
	public static RosbridgeRequest publishPose(String topic, float[] point, float[] quat) throws JSONException {	
		JSONObject poseMsg = pose(vector3(point[0], point[1], point[2]), quaternion(quat[0], quat[1], quat[2], quat[3]));
		return publish(topic, poseMsg);
	}

}
