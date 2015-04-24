package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.RigidBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class ElasticContact extends Constraint {

	protected final static Vector3f tangentFrictionImpulse = new Vector3f();
	protected final static Vector3f rV = new Vector3f();

	protected final static float percent = 0.8f;
	protected final static float slop = 0.05f;
	// protected final static float percent = 1.0f;
	// protected final static float slop = 0.0f;

	protected final Vector3f pointA = new Vector3f();
	protected final Vector3f pointB = new Vector3f();
	protected Vector3f normal = new Vector3f();
	protected float distance;

	public ElasticContact(final StaticBody a, final DynamicBody b,
			final Contact contact) throws Exception {
		reset(a, b, contact);
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
			final Contact contact) throws Exception {
		setBodies(a, b);
		normal.set(contact.getNormal());
		distance = contact.getDistance();
		pointA.setSubtract(contact.getPointA(), a.getPosition());
		pointB.setSubtract(contact.getPointB(), b.getPosition());
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
		} else {// TODO
			// dynamicContact(uInfo);
			rigidContact(uInfo);
		}
	}// http://en.wikipedia.org/wiki/Collision_response

	private void staticContact(final IUpdateInfo uInfo) {
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			dynB.getPositionCorrection().addScaled(normal, percent * distance);
		}
		final Vector3f dynBlV = dynB.getLinearVelocity();
		final Vector3f relVel = dynB instanceof RigidBody ? tmp1.setAdd(dynBlV,
				tmp1.setCross(((RigidBody) dynB).getAngularVelocity(), pointB))
				: dynBlV;
		final float velAlongNormal = relVel.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		final Material mA = stcA.getMaterial();
		final Material mB = dynB.getMaterial();
		final float jn;
		if (dynB instanceof RigidBody) {
			// tmp2.setCross(pointB, normal).multiply(
			// ((RigidBody) dynB).getInverseInertiaTensor());
			tmp2.multiplyM3V(((RigidBody) dynB).getInverseInertiaTensor(), 0,
					tmp2.setCross(pointB, normal));
			tmp21.setCross(tmp2, pointB);
			jn = (-(1 + Math.max(mA.getRestitution(), mB.getRestitution())) * velAlongNormal)
					/ (dynB.getInverseMass() + tmp21.dot(normal));
			dynBlV.addScaled(normal, jn * dynB.getInverseMass());
			((RigidBody) dynB).getAngularVelocity().addScaled(tmp2, jn);
		} else {
			jn = -(1 + Math.max(mA.getRestitution(), mB.getRestitution()))
					* velAlongNormal;
			dynBlV.addScaled(normal, jn);
		}
		if (velAlongNormal != 0) {
			tangentFrictionImpulse.setSubtractScaled(relVel, normal,
					relVel.dot(normal)).norm();
			final float jt = dynBlV.dot(tangentFrictionImpulse);
			if (jt != 0) {
				final float clampedStaticFriction = clamp(
						mA.getStaticFriction(), mB.getStaticFriction());
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
	}

	private void dynamicContact(final IUpdateInfo uInfo) {
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			rV.setScale(
					normal,
					percent
							* (distance / (dynA.getInverseMass() + dynB
									.getInverseMass())));
			dynA.getPositionCorrection().subtractScaled(rV,
					dynA.getInverseMass());
			dynB.getPositionCorrection().addScaled(rV, dynB.getInverseMass());
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

	static Vector3f tmp1 = new Vector3f();
	static Vector3f tmp11 = new Vector3f();
	static Vector3f tmp2 = new Vector3f();
	static Vector3f tmp21 = new Vector3f();

	private void rigidContact(final IUpdateInfo uInfo) {
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			rV.setScale(
					normal,
					percent
							* (distance / (dynA.getInverseMass() + dynB
									.getInverseMass())));
			dynA.getPositionCorrection().subtractScaled(rV,
					dynA.getInverseMass());
			dynB.getPositionCorrection().addScaled(rV, dynB.getInverseMass());
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

		if (dynA instanceof RigidBody) {
			// tmp1.setCross(pointA, normal).multiply(
			// ((RigidBody) dynA).getInverseInertiaTensor());
			tmp1.multiplyM3V(((RigidBody) dynA).getInverseInertiaTensor(), 0,
					tmp1.setCross(pointA, normal));
			tmp11.setCross(tmp1, pointA);
		} else
			tmp11.set(0, 0, 0);
		if (dynB instanceof RigidBody) {
			// tmp2.setCross(pointB, normal).multiply(
			// ((RigidBody) dynB).getInverseInertiaTensor());
			tmp2.multiplyM3V(((RigidBody) dynB).getInverseInertiaTensor(), 0,
					tmp2.setCross(pointB, normal));
			tmp21.setCross(tmp2, pointB);
			tmp11.add(tmp21);
		}
		float jn = (-(1 + Math.max(mA.getRestitution(), mB.getRestitution())) * velAlongNormal)
				/ (dynA.getInverseMass() + dynB.getInverseMass() + tmp11
						.dot(normal));
		dynAlV.subtractScaled(normal, jn * dynA.getInverseMass());
		dynBlV.addScaled(normal, jn * dynB.getInverseMass());
		if (dynA instanceof RigidBody)
			((RigidBody) dynA).getAngularVelocity().subtractScaled(tmp1, jn);
		if (dynB instanceof RigidBody)
			((RigidBody) dynB).getAngularVelocity().addScaled(tmp2, jn);

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
