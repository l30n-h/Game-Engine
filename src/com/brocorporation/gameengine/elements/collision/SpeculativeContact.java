package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class SpeculativeContact extends Constraint {

	protected final static Vector3f tangentFrictionImpulse = new Vector3f();
	protected final static Vector3f rV = new Vector3f();

	protected Vector3f normal = new Vector3f();
	protected float distance;
	protected float impulse;
	protected float inverseInverseMassSum;
	protected float clampedStaticFriction = -1;
	protected float clampedDynamicFriction = -1;

	public SpeculativeContact(final StaticBody a, final DynamicBody b,
			final Vector3f pNormal, final float pDistance) throws Exception {
		reset(a, b, pNormal, pDistance);
	}

	@Override
	public void setBodies(final DynamicBody a, final DynamicBody b)
			throws Exception {
		super.setBodies(a, b);
		if (stcA == null) {
			inverseInverseMassSum = 1F / (dynA.getInverseMass() + dynB
					.getInverseMass());
			if (normal.y < -0.9f) {
				dynA.isOnGround(true);
			}
		}
		if (normal.y > 0.9f) {
			dynB.isOnGround(true);
		}
	}

	@Override
	public void setBodies(final StaticBody a, final DynamicBody b)
			throws Exception {
		if (a instanceof DynamicBody) {
			setBodies((DynamicBody) a, b);
		} else {
			super.setBodies(a, b);
		}
		if (normal.y > 0.9f) {
			dynB.isOnGround(true);
		}
	}

	public void reset(final StaticBody a, final DynamicBody b,
			final Vector3f pNormal, final float pDistance) throws Exception {
		setBodies(a, b);
		normal.set(pNormal);
		distance = pDistance;
		impulse = 0;
		clampedStaticFriction = -1;
		clampedDynamicFriction = -1;
	}

	@Override
	public void reset() {
		super.reset();
		distance = 0;
		impulse = 0;
		inverseInverseMassSum = 0;
		clampedStaticFriction = -1;
		clampedDynamicFriction = -1;
	}

	@Override
	public void solve(final IUpdateInfo uInfo) {
		if (stcA != null) {
			staticContact(uInfo);
		} else {
			dynamicContact(uInfo);
		}
	}

	private void staticContact(final IUpdateInfo uInfo) {
		if (distance < 0)
			return;
		final Vector3f dynBlV = dynB.getLinearVelocity();
		final float remove = dynBlV.dot(normal) + distance
				* uInfo.getInverseRate();
		final float newImpulse = Math.min(remove + impulse, 0);
		final float jn = impulse - newImpulse;
		impulse = newImpulse;
		if (jn != 0) {
			dynBlV.addScaled(normal, jn);
			tangentFrictionImpulse.setSubtractScaled(dynBlV, normal,
					dynBlV.dot(normal)).norm();
			final float jt = dynBlV.dot(tangentFrictionImpulse);
			if (jt != 0) {
				if (clampedStaticFriction < 0) {
					clampedStaticFriction = clamp(stcA.getMaterial()
							.getStaticFriction(), dynB.getMaterial()
							.getStaticFriction());
				}
				if (jt < jn * clampedStaticFriction) {
					dynBlV.subtractScaled(tangentFrictionImpulse, jt);
				} else {
					if (clampedDynamicFriction < 0) {
						clampedDynamicFriction = clamp(stcA.getMaterial()
								.getDynamicFriction(), dynB.getMaterial()
								.getDynamicFriction());
					}
					dynBlV.subtractScaled(tangentFrictionImpulse, jn
							* clampedDynamicFriction);
				}
			}
		}
	}

	private void dynamicContact(final IUpdateInfo uInfo) {
		if (distance < 0)
			return;
		final Vector3f dynAlV = dynA.getLinearVelocity();
		final Vector3f dynBlV = dynB.getLinearVelocity();
		rV.setSubtract(dynBlV, dynAlV);
		final float remove = rV.dot(normal) + distance * uInfo.getInverseRate();
		final float newImpulse = Math.min((remove * inverseInverseMassSum)
				+ impulse, 0);
		final float jn = impulse - newImpulse;
		impulse = newImpulse;
		if (jn != 0) {
			dynAlV.subtractScaled(normal, jn * dynA.getInverseMass());
			dynBlV.addScaled(normal, jn * dynB.getInverseMass());

			rV.setSubtract(dynBlV, dynAlV);
			tangentFrictionImpulse
					.setSubtractScaled(rV, normal, rV.dot(normal)).norm();
			float jt = rV.dot(tangentFrictionImpulse);
			if (jt != 0) {
				jt *= inverseInverseMassSum;
				if (clampedStaticFriction < 0) {
					clampedStaticFriction = clamp(dynA.getMaterial()
							.getStaticFriction(), dynB.getMaterial()
							.getStaticFriction());
				}
				if (jt < jn * clampedStaticFriction) {
					dynAlV.addScaled(tangentFrictionImpulse,
							jt * dynA.getInverseMass());
					dynBlV.subtractScaled(tangentFrictionImpulse,
							jt * dynB.getInverseMass());
				} else {
					if (clampedDynamicFriction < 0) {
						final Material mA = dynA.getMaterial();
						final Material mB = dynB.getMaterial();
						clampedDynamicFriction = clamp(mA.getDynamicFriction(),
								mB.getDynamicFriction());
					}
					final float fs = clampedDynamicFriction * jn;
					dynAlV.addScaled(tangentFrictionImpulse,
							fs * dynA.getInverseMass());
					dynBlV.subtractScaled(tangentFrictionImpulse,
							fs * dynB.getInverseMass());
					tangentFrictionImpulse.scale(jn * clampedDynamicFriction);
				}
			}
		}
	}

	protected static float clamp(final float frictionA, final float frictionB) {
		return (float) Math.sqrt(frictionA * frictionA + frictionB * frictionB);
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof SpeculativeContact) {
			final SpeculativeContact c = (SpeculativeContact) o;
			if (stcA != null) {
				return (stcA == c.stcA && dynB == c.dynB);
			} else {
				return (dynA == c.dynA && dynB == c.dynB)
						|| (dynA == c.dynB && dynB == c.dynA);
			}
		}
		return false;
	}

	@Override
	public int hashCode() {
		return Math.abs((stcA != null ? stcA.hashCode() : dynA.hashCode())
				- dynB.hashCode());
	}
}
