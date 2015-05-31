package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class FollowConstraint extends Constraint {

	protected float distance;
	protected boolean stop = false;

	public FollowConstraint(final StaticBody a, final DynamicBody b,
			final float d) throws Exception {
		setBodies(a, b);
		distance = d;
	}

	public void setDistance(final float d) {
		distance = d;
	}

	public float getDistance() {
		return distance;
	}

	@Override
	public void solve(final IUpdateInfo uInfo) {
		final Vector3f posA = bodyA.getPosition();
		final Vector3f posB = bodyB.getPosition();
		final float x = posA.x - posB.x;
		final float y = posA.y - posB.y;
		final float z = posA.z - posB.z;
		final float length = Vector3f.length(x, y, z);
		if (length > distance) {
			final float s = (length - distance) * uInfo.getInverseRate();
			((DynamicBody) bodyB).getLinearVelocity().addScaled(x, y, z, s);
			stop = true;
		} else {
			if (stop) {
				((DynamicBody) bodyB).getLinearVelocity().set(0, 0, 0);
				stop = false;
			}
		}
	}
}
