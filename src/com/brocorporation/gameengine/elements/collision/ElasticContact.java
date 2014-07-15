package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class ElasticContact extends Constraint {

	protected final static Vector3f tangentFrictionImpulse = new Vector3f();
	protected final static Vector3f rV = new Vector3f();

	protected final static float percent = 0.5f;
	protected final static float slop = 0.05f;
//	protected final static float percent = 1;
//	protected final static float slop = 0.0f;

	protected Vector3f normal = new Vector3f();
	protected float distance;

	public ElasticContact(final StaticBody a, final DynamicBody b,
			final Vector3f pNormal, final float pDistance) throws Exception {
		reset(a, b, pNormal, pDistance);
	}

	@Override
	public void setBodies(final DynamicBody a, final DynamicBody b)
			throws Exception {
		super.setBodies(a, b);
		if (stcA == null) {
			if (normal.y < -0.25f) {
				dynA.isOnGround(true);
			}
		}
		if (normal.y > 0.25f) {
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
		if (normal.y > 0.25f) {
			dynB.isOnGround(true);
		}
	}

	public void reset(final StaticBody a, final DynamicBody b,
			final Vector3f pNormal, final float pDistance) throws Exception {
		setBodies(a, b);
		normal.set(pNormal);
		distance = pDistance;
	}

	@Override
	public void reset() {
		super.reset();
		distance = 0;
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
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			dynB.getPositionCorrection().addScaled(normal, percent * distance);
		}
		final Vector3f dynBlV = dynB.getLinearVelocity();
		final float velAlongNormal = dynBlV.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		final Material mA = stcA.getMaterial();
		final Material mB = dynB.getMaterial();
		final float jn = -(1 + Math.max(mA.getRestitution(),
				mB.getRestitution()))
				* velAlongNormal;
		dynBlV.addScaled(normal, jn);
		tangentFrictionImpulse.setSubtractScaled(dynBlV, normal,
				dynBlV.dot(normal)).norm();
		final float jt = dynBlV.dot(tangentFrictionImpulse);
		if (jt != 0) {
			final float clampedStaticFriction = clamp(mA.getStaticFriction(),
					mB.getStaticFriction());
			if (jt < jn * clampedStaticFriction) {
				dynBlV.subtractScaled(tangentFrictionImpulse, jt);
			} else {
				final float clampedDynamicFriction = clamp(
						mA.getDynamicFriction(), mB.getDynamicFriction());
				dynBlV.subtractScaled(tangentFrictionImpulse, jn
						* clampedDynamicFriction);
			}
		}
	}

	// jn = (-(1+e)*rv)/(inverseMassSum+(iA*(rAxn))xrA+(iB*(rBxn))xrB);
	// wA = wA-iA*(rAx(jn*n)
	// wB = wB+iB*(rBx(jn*n)
	// lA = lA-j/mA*n
	// lB = lB+j/mB*n

	private void dynamicContact(final IUpdateInfo uInfo) {
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			rV.setScale(normal, percent * distance);
			dynA.getPositionCorrection().subtract(rV);
			dynB.getPositionCorrection().add(rV);
		}
		final Vector3f dynAlV = dynA.getLinearVelocity();
		final Vector3f dynBlV = dynB.getLinearVelocity();
		rV.setSubtract(dynBlV, dynAlV);
		final float velAlongNormal = rV.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		final float inverseInverseMassSum = 1F / (dynA.getInverseMass() + dynB
				.getInverseMass());
		final Material mA = dynA.getMaterial();
		final Material mB = dynB.getMaterial();
		final float jn = -(1 + Math.max(mA.getRestitution(),
				mB.getRestitution()))
				* velAlongNormal * inverseInverseMassSum;
		dynAlV.subtractScaled(normal, jn * dynA.getInverseMass());
		dynBlV.addScaled(normal, jn * dynB.getInverseMass());

		rV.setSubtract(dynBlV, dynAlV);
		tangentFrictionImpulse.setSubtractScaled(rV, normal, rV.dot(normal))
				.norm();

		float jt = rV.dot(tangentFrictionImpulse);
		if (jt != 0) {
			jt *= inverseInverseMassSum;
			final float clampedStaticFriction = clamp(mA.getStaticFriction(),
					mB.getStaticFriction());
			if (jt < jn * clampedStaticFriction) {
				dynAlV.addScaled(tangentFrictionImpulse,
						jt * dynA.getInverseMass());
				dynBlV.subtractScaled(tangentFrictionImpulse,
						jt * dynB.getInverseMass());
			} else {
				final float fs = clamp(mA.getDynamicFriction(),
						mB.getDynamicFriction())
						* jn;
				dynAlV.addScaled(tangentFrictionImpulse,
						fs * dynA.getInverseMass());
				dynBlV.subtractScaled(tangentFrictionImpulse,
						fs * dynB.getInverseMass());
			}
		}
	}

	protected static float clamp(final float frictionA, final float frictionB) {
		return (float) Math.sqrt(frictionA * frictionA + frictionB * frictionB);
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof ElasticContact) {
			final ElasticContact c = (ElasticContact) o;
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
