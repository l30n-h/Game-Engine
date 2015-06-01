package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.RigidBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class SpeculativeContact extends Constraint {

	protected final static Vector3f tangent = new Vector3f();
	protected final static Vector3f rV = new Vector3f();

	protected final Vector3f pointA = new Vector3f();
	protected final Vector3f pointB = new Vector3f();
	protected Vector3f normal = new Vector3f();
	protected float distance;
	protected float impulse;
	protected float inverseInverseMassSum;
	protected float clampedStaticFriction = -1;
	protected float clampedDynamicFriction = -1;

	public SpeculativeContact(final StaticBody a, final DynamicBody b,
			final Contact contact) throws Exception {
		reset(a, b, contact);
	}

	public void reset(final StaticBody a, final DynamicBody b,
			final Contact contact) throws Exception {
		setBodies(a, b);
		if (bodyA instanceof DynamicBody) {
			inverseInverseMassSum = 1F / (bodyA.getInverseMass() + bodyB
					.getInverseMass());
			if (normal.y < -0.25f) {
				((DynamicBody) bodyA).isOnGround(true);
			}
		}
		if (normal.y > 0.25f) {
			((DynamicBody) bodyB).isOnGround(true);
		}
		normal.set(contact.getNormal());
		distance = contact.getDistance();
		pointA.setSubtract(contact.getPointA(), a.getPosition());
		pointB.setSubtract(contact.getPointB(), b.getPosition());
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
				staticContact2(uInfo);
			}
		} else {
			// dynamicContact(uInfo);
			rigidContact(uInfo);
		}
	}

	static Vector3f tmp1 = new Vector3f();// TODO
	static Vector3f tmp11 = new Vector3f();
	static Vector3f tmp2 = new Vector3f();
	static Vector3f tmp21 = new Vector3f();
	static boolean frictional = true;// TODO

	private void staticContact2(final IUpdateInfo uInfo) {
		if (distance < 0)
			distance = 0;
		DynamicBody bodyB = (DynamicBody) this.bodyB;
		final Vector3f dynBlV = bodyB.getLinearVelocity();
		boolean isRigidB = bodyB instanceof RigidBody;
		Vector3f relVel = isRigidB ? rV.setAdd(dynBlV,
				rV.setCross(((RigidBody) bodyB).getAngularVelocity(), pointB))
				: dynBlV;
		float velAlongNormal = relVel.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		if (isRigidB) {
			final float remove = relVel.dot(normal) + distance
					* uInfo.getInverseRate();
			final float newImpulse = Math.min((remove * bodyB.getMass())
					+ impulse, 0);
			final float jn = impulse - newImpulse;
			impulse = newImpulse;
			final float[] itB = ((RigidBody) bodyB).getInverseInertiaTensor();
			tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, normal));
			tmp11.setCross(tmp2, pointB);
			dynBlV.addScaled(normal, bodyB.getInverseMass() * jn);
			((RigidBody) bodyB).getAngularVelocity().addScaled(tmp2, jn);
			if (frictional) {
				relVel = rV.setAdd(dynBlV, rV.setCross(
						((RigidBody) bodyB).getAngularVelocity(), pointB));
				velAlongNormal = relVel.dot(normal);
				tangent.setSubtractScaled(relVel, normal, velAlongNormal)
						.norm();
				tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, tangent));
				tmp11.setCross(tmp2, pointB);
				float jt = -relVel.dot(tangent);
				if (jt != 0) {
					jt /= (bodyB.getInverseMass() + tmp11.dot(tangent));
					if (clampedStaticFriction < 0) {
						clampedStaticFriction = clamp(bodyA.getMaterial()
								.getStaticFriction(), bodyB.getMaterial()
								.getStaticFriction());
					}
					if (Math.abs(jt) < jn * clampedStaticFriction) {
						tmp21.setScale(tangent, jt);
					} else {
						if (clampedDynamicFriction < 0) {
							clampedDynamicFriction = clamp(bodyA.getMaterial()
									.getDynamicFriction(), bodyB.getMaterial()
									.getDynamicFriction());
						}
						tmp21.setScale(tangent, -clampedDynamicFriction * jn);
					}
					tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, tmp21));
					dynBlV.addScaled(tmp21, bodyB.getInverseMass());
					((RigidBody) bodyB).getAngularVelocity().add(tmp2);
				}
			}
		} else {
			final float remove = relVel.dot(normal) + distance
					* uInfo.getInverseRate();
			final float newImpulse = Math.min(remove + impulse, 0);
			final float jn = impulse - newImpulse;
			impulse = newImpulse;
			dynBlV.addScaled(normal, jn);
			if (frictional) {
				relVel = dynBlV;
				velAlongNormal = relVel.dot(normal);
				tangent.setSubtractScaled(relVel, normal, velAlongNormal)
						.norm();
				final float jt = -relVel.dot(tangent);
				if (jt != 0) {
					if (clampedStaticFriction < 0) {
						clampedStaticFriction = clamp(bodyA.getMaterial()
								.getStaticFriction(), bodyB.getMaterial()
								.getStaticFriction());
					}
					if (Math.abs(jt) < jn * clampedStaticFriction) {
						dynBlV.addScaled(tangent, jt);
					} else {
						if (clampedDynamicFriction < 0) {
							clampedDynamicFriction = clamp(bodyA.getMaterial()
									.getDynamicFriction(), bodyB.getMaterial()
									.getDynamicFriction());
						}
						dynBlV.addScaled(tangent, -clampedDynamicFriction * jn);
					}
				}
			}
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
			tangent.setSubtractScaled(dynBlV, normal, dynBlV.dot(normal))
					.norm();
			final float jt = dynBlV.dot(tangent);
			if (jt != 0) {
				if (clampedStaticFriction < 0) {
					clampedStaticFriction = clamp(bodyA.getMaterial()
							.getStaticFriction(), bodyB.getMaterial()
							.getStaticFriction());
				}
				if (jt < jn * clampedStaticFriction) {
					dynBlV.subtractScaled(tangent, jt);
				} else {
					if (clampedDynamicFriction < 0) {
						clampedDynamicFriction = clamp(bodyA.getMaterial()
								.getDynamicFriction(), bodyB.getMaterial()
								.getDynamicFriction());
					}
					dynBlV.subtractScaled(tangent, jn * clampedDynamicFriction);
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
			tangent.setSubtractScaled(rV, normal, rV.dot(normal)).norm();
			float jt = rV.dot(tangent);
			if (jt != 0) {
				jt *= inverseInverseMassSum;
				if (clampedStaticFriction < 0) {
					clampedStaticFriction = clamp(bodyA.getMaterial()
							.getStaticFriction(), bodyB.getMaterial()
							.getStaticFriction());
				}
				if (jt < jn * clampedStaticFriction) {
					dynAlV.addScaled(tangent, jt * bodyA.getInverseMass());
					dynBlV.subtractScaled(tangent, jt * bodyB.getInverseMass());
				} else {
					if (clampedDynamicFriction < 0) {
						final Material mA = bodyA.getMaterial();
						final Material mB = bodyB.getMaterial();
						clampedDynamicFriction = clamp(mA.getDynamicFriction(),
								mB.getDynamicFriction());
					}
					final float fs = clampedDynamicFriction * jn;
					dynAlV.addScaled(tangent, fs * bodyA.getInverseMass());
					dynBlV.subtractScaled(tangent, fs * bodyB.getInverseMass());
					tangent.scale(jn * clampedDynamicFriction);
				}
			}
		}
	}

	private void rigidContact(final IUpdateInfo uInfo) {
		if (distance < 0)
			distance = 0;
		final DynamicBody bodyA = (DynamicBody) this.bodyA;
		final DynamicBody bodyB = (DynamicBody) this.bodyB;
		final Vector3f dynAlV = bodyA.getLinearVelocity();
		final Vector3f dynBlV = bodyB.getLinearVelocity();
		boolean isRigidA = bodyA instanceof RigidBody;
		boolean isRigidB = bodyB instanceof RigidBody;
		rV.setSubtract(
				isRigidB ? tmp2.setAdd(dynBlV, tmp2.setCross(
						((RigidBody) bodyB).getAngularVelocity(), pointB))
						: dynBlV,
				isRigidA ? tmp1.setAdd(dynAlV, tmp1.setCross(
						((RigidBody) bodyA).getAngularVelocity(), pointA))
						: dynAlV);
		float velAlongNormal = rV.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		final Material mA = bodyA.getMaterial();
		final Material mB = bodyB.getMaterial();
		if (isRigidA) {
			tmp1.multiplyM3V(((RigidBody) bodyA).getInverseInertiaTensor(), 0,
					tmp1.setCross(pointA, normal));
			tmp11.setCross(tmp1, pointA);
		} else {
			tmp11.set(0, 0, 0);
		}
		if (isRigidB) {
			tmp2.multiplyM3V(((RigidBody) bodyB).getInverseInertiaTensor(), 0,
					tmp2.setCross(pointB, normal));
			tmp21.setCross(tmp2, pointB);
			tmp11.add(tmp21);
		}
		final float remove = rV.dot(normal) + distance * uInfo.getInverseRate();
		final float newImpulse = Math.min((remove * inverseInverseMassSum)
				+ impulse, 0);
		final float jn = impulse - newImpulse;
		impulse = newImpulse;
		dynAlV.subtractScaled(normal, bodyA.getInverseMass() * jn);
		dynBlV.addScaled(normal, bodyB.getInverseMass() * jn);
		if (isRigidA)
			((RigidBody) bodyA).getAngularVelocity().subtractScaled(tmp1, jn);
		if (isRigidB)
			((RigidBody) bodyB).getAngularVelocity().addScaled(tmp2, jn);

		if (frictional) {
			rV.setSubtract(
					isRigidB ? tmp2.setAdd(dynBlV, tmp2.setCross(
							((RigidBody) bodyB).getAngularVelocity(), pointB))
							: dynBlV,
					isRigidA ? tmp1.setAdd(dynAlV, tmp1.setCross(
							((RigidBody) bodyA).getAngularVelocity(), pointA))
							: dynAlV);
			velAlongNormal = rV.dot(normal);
			tangent.setSubtractScaled(rV, normal, velAlongNormal).norm();
			float jt = -rV.dot(tangent);
			if (jt != 0) {
				if (isRigidA) {
					tmp1.multiplyM3V(
							((RigidBody) bodyA).getInverseInertiaTensor(), 0,
							tmp1.setCross(pointA, tangent));
					tmp11.setCross(tmp1, pointA);
				} else {
					tmp11.set(0, 0, 0);
				}
				if (isRigidB) {
					tmp2.multiplyM3V(
							((RigidBody) bodyB).getInverseInertiaTensor(), 0,
							tmp2.setCross(pointB, tangent));
					tmp21.setCross(tmp2, pointB);
					tmp11.add(tmp21);
				}
				jt /= (bodyA.getInverseMass() + bodyB.getInverseMass() + tmp11
						.dot(tangent));
				if (Math.abs(jt) < jn
						* clamp(mA.getStaticFriction(), mB.getStaticFriction())) {
					tmp21.setScale(tangent, jt);
				} else {
					tmp21.setScale(
							tangent,
							-clamp(mA.getDynamicFriction(),
									mB.getDynamicFriction())
									* jn);
				}
				dynAlV.subtractScaled(tmp21, bodyA.getInverseMass());
				dynBlV.addScaled(tmp21, bodyB.getInverseMass());
				if (isRigidA) {
					tmp1.multiplyM3V(
							((RigidBody) bodyA).getInverseInertiaTensor(), 0,
							tmp1.setCross(pointA, tmp21));
					((RigidBody) bodyA).getAngularVelocity().subtract(tmp1);
				}
				if (isRigidB) {
					tmp2.multiplyM3V(
							((RigidBody) bodyB).getInverseInertiaTensor(), 0,
							tmp2.setCross(pointB, tmp21));
					((RigidBody) bodyB).getAngularVelocity().add(tmp2);
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
