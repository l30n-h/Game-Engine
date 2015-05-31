package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.StaticBody;

public abstract class Constraint {

	protected StaticBody bodyA;
	protected StaticBody bodyB;

	public Constraint() {
	}

	protected void setBodies(final StaticBody a, final StaticBody b)
			throws Exception {
		if (b != null && a != null) {
			bodyB = b;
			bodyA = a;
		} else {
			throw new Exception();
		}
	}

	public void reset() {
		bodyB = null;
		bodyA = null;
	}

	public abstract void solve(final IUpdateInfo uInfo);
}
