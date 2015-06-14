package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class Contact {

	public final static Contact DEFAULT = new Contact();

	protected Vector3f pointA = new Vector3f();
	protected Vector3f pointB = new Vector3f();
	protected final Vector3f normal = new Vector3f();
	protected float distance;

	public Contact() {
	}

	public void setDistance(float d) {
		distance = d;
	}

	public Vector3f getPointA() {
		return pointA;
	}

	public Vector3f getPointB() {
		return pointB;
	}

	public Vector3f getNormal() {
		return normal;
	}

	public float getDistance() {
		return distance;
	}

	public void swap() {
		normal.invert();
		final Vector3f tmp = pointA;
		pointA = pointB;
		pointB = tmp;
	}
}
