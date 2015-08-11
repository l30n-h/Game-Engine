package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class Contact {

	public final static Contact DEFAULT = new Contact();

	protected Vector3f worldA = new Vector3f();
	protected Vector3f worldB = new Vector3f();
	protected final Vector3f normal = new Vector3f();
	protected float distance;

	public Contact() {
	}

	public void setDistance(float d) {
		distance = d;
	}

	public Vector3f getPointA() {
		return worldA;
	}

	public Vector3f getPointB() {
		return worldB;
	}

	public Vector3f getNormal() {
		return normal;
	}

	public float getDistance() {
		return distance;
	}

	public void swap() {
		normal.invert();
		final Vector3f tmp = worldA;
		worldA = worldB;
		worldB = tmp;
	}
}
