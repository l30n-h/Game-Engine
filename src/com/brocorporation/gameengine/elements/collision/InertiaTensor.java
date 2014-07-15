package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class InertiaTensor {

	public static void getCuboid(Vector3f inertiaTensor, float mass,
			Vector3f halfsize) {
		final Vector3f hs = halfsize;
		final float ww = hs.x * hs.x;
		final float hh = hs.y * hs.y;
		final float dd = hs.z * hs.z;
		final float s = 0.3333333F * mass;
		inertiaTensor.set(s * (hh + dd), s * (ww + dd), s * (ww + hh));
	}
}
