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

	public void reset(final StaticBody a, final DynamicBody b,
			final Vector3f pNormal, final float pDistance) throws Exception {
		setBodies(a, b);
		if (a instanceof DynamicBody) {
			inverseInverseMassSum = 1F / bodyA.getInverseMass()
					+ bodyB.getInverseMass();
			if (normal.y < -0.9f) {
				((DynamicBody) a).isOnGround(true);
			}
		}
		if (normal.y > 0.9f) {
			((DynamicBody) bodyB).isOnGround(true);
		}
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
		if (bodyA.isStatic()) {
			if (!bodyB.isStatic()) {
				staticContact(uInfo);
			}
		} else {
			dynamicContact(uInfo);
		}
	}

	private void staticContact(final IUpdateInfo uInfo) {
		if (distance < 0)
			return;
		DynamicBody bodyB = (DynamicBody) this.bodyB;
		final Vector3f dynBlV = bodyB.getLinearVelocity();
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
					clampedStaticFriction = clamp(bodyA.getMaterial()
							.getStaticFriction(), bodyB.getMaterial()
							.getStaticFriction());
				}
				if (jt < jn * clampedStaticFriction) {
					dynBlV.subtractScaled(tangentFrictionImpulse, jt);
				} else {
					if (clampedDynamicFriction < 0) {
						clampedDynamicFriction = clamp(bodyA.getMaterial()
								.getDynamicFriction(), bodyB.getMaterial()
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
		final DynamicBody bodyA = (DynamicBody) this.bodyA;
		final DynamicBody bodyB = (DynamicBody) this.bodyB;
		final Vector3f dynAlV = bodyA.getLinearVelocity();
		final Vector3f dynBlV = bodyB.getLinearVelocity();
		rV.setSubtract(dynBlV, dynAlV);
		final float remove = rV.dot(normal) + distance * uInfo.getInverseRate();
		final float newImpulse = Math.min((remove * inverseInverseMassSum)
				+ impulse, 0);
		final float jn = impulse - newImpulse;
		impulse = newImpulse;
		if (jn != 0) {
			dynAlV.subtractScaled(normal, jn * bodyA.getInverseMass());
			dynBlV.addScaled(normal, jn * bodyB.getInverseMass());

			rV.setSubtract(dynBlV, dynAlV);
			tangentFrictionImpulse
					.setSubtractScaled(rV, normal, rV.dot(normal)).norm();
			float jt = rV.dot(tangentFrictionImpulse);
			if (jt != 0) {
				jt *= inverseInverseMassSum;
				if (clampedStaticFriction < 0) {
					clampedStaticFriction = clamp(bodyA.getMaterial()
							.getStaticFriction(), bodyB.getMaterial()
							.getStaticFriction());
				}
				if (jt < jn * clampedStaticFriction) {
					dynAlV.addScaled(tangentFrictionImpulse,
							jt * bodyA.getInverseMass());
					dynBlV.subtractScaled(tangentFrictionImpulse,
							jt * bodyB.getInverseMass());
				} else {
					if (clampedDynamicFriction < 0) {
						final Material mA = bodyA.getMaterial();
						final Material mB = bodyB.getMaterial();
						clampedDynamicFriction = clamp(mA.getDynamicFriction(),
								mB.getDynamicFriction());
					}
					final float fs = clampedDynamicFriction * jn;
					dynAlV.addScaled(tangentFrictionImpulse,
							fs * bodyA.getInverseMass());
					dynBlV.subtractScaled(tangentFrictionImpulse,
							fs * bodyB.getInverseMass());
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
			return (bodyA == c.bodyA && bodyB == c.bodyB)
					|| (bodyA == c.bodyB && bodyB == c.bodyA);
		}
		return false;
	}

	@Override
	public int hashCode() {
		return Math.abs(bodyA.hashCode() - bodyB.hashCode());
	}
}
