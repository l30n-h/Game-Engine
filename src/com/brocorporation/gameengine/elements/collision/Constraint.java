package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;

public abstract class Constraint {

	protected StaticBody stcA;
	protected DynamicBody dynA;
	protected DynamicBody dynB;

	public Constraint() {
	}

	protected void setBodies(final DynamicBody a, final DynamicBody b)
			throws Exception {
		if (b != null && a != null) {
			if (a.getInverseMass() == DynamicBody.INFINITY_MASS
					&& b.getInverseMass() == DynamicBody.INFINITY_MASS) {
				throw new Exception();
			} else {
				dynA = a;
				dynB = b;
				stcA = null;
			}
		} else {
			throw new Exception();
		}
	}

	protected void setBodies(final StaticBody a, final DynamicBody b)
			throws Exception {
		if (a instanceof DynamicBody) {
			setBodies((DynamicBody) a, b);
		} else {
			dynA = null;
			stcA = a;
			dynB = b;
			if ((stcA == null && dynB == null)
					|| dynB.getInverseMass() == DynamicBody.INFINITY_MASS)
				throw new Exception();
		}
	}

	public void reset() {
		dynA = null;
		dynB = null;
		stcA = null;
	}

	public abstract void solve(final IUpdateInfo uInfo);
}
