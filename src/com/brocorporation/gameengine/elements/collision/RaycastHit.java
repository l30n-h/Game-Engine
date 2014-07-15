package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class RaycastHit {

	public final static RaycastHit DEFAULT = new RaycastHit();

	protected final Vector3f point = new Vector3f();
	protected float scalar;

	public Vector3f getPoint() {
		return point;
	}

	public void setScalar(float pScalar) {
		scalar = pScalar;
	}

	public float getScalar() {
		return scalar;
	}
}
