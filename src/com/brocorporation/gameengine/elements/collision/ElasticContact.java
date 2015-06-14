package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.RigidBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class ElasticContact extends Constraint {

	protected final static Vector3f tangent = new Vector3f();
	protected final static Vector3f rV = new Vector3f();

	protected final static float percent = 0.8f;
	protected final static float slop = 0.05f;
//	 protected final static float percent = 1.0f;
//	 protected final static float slop = 0.0f;

	protected final Vector3f pointA = new Vector3f();
	protected final Vector3f pointB = new Vector3f();
	protected Vector3f normal = new Vector3f();
	protected float distance;

	static Vector3f tmp1 = new Vector3f();// TODO
	static Vector3f tmp11 = new Vector3f();
	static Vector3f tmp2 = new Vector3f();
	static Vector3f tmp21 = new Vector3f();

	public ElasticContact(final StaticBody a, final DynamicBody b,
			final Contact contact) throws Exception {
		reset(a, b, contact);
	}

	public void reset(final StaticBody a, final DynamicBody b,
			final Contact contact) throws Exception {
		setBodies(a, b);
		if (bodyA instanceof DynamicBody) {
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
	}

	@Override
	public void reset() {
		super.reset();
		distance = 0;
	}

	@Override
	public void solve(final IUpdateInfo uInfo) {
		if (bodyA.isStatic()) {
			staticContact(uInfo);
		} else {// TODO
			// dynamicContact(uInfo);
			rigidContact(uInfo);
		}
	}// http://en.wikipedia.org/wiki/Collision_response

	static boolean frictional = true;// TODO

//	ContactPoint
//	bodyA
//	bodyB
//	Vector pointA
//	Vector pointB
//	Vector localA
//	Vector localB
//	Vector I(pointxnormal)
//	float impulse
//--------------------------
//	ContactSet
//	Vector normal
//	float stcFriction
//	float dynFriction
	
	private void staticContact(final IUpdateInfo uInfo) {
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
		if ((distance += slop) < 0) {
			bodyB.getPositionCorrection().addScaled(normal, percent * distance);
		}
		final Material mA = bodyA.getMaterial();
		final Material mB = bodyB.getMaterial();
		final float jn;
		if (isRigidB) {
			final float[] itB = ((RigidBody) bodyB).getInverseInertiaTensor();
			tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, normal));
			tmp11.setCross(tmp2, pointB);
			jn = (-(1 + Math.max(mA.getRestitution(), mB.getRestitution())) * velAlongNormal)
					/ (bodyB.getInverseMass() + tmp11.dot(normal));
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
					if (Math.abs(jt) < jn
							* clamp(mA.getStaticFriction(),
									mB.getStaticFriction())) {
						tmp21.setScale(tangent, jt);
					} else {
						tmp21.setScale(
								tangent,
								-clamp(mA.getDynamicFriction(),
										mB.getDynamicFriction())
										* jn);
					}
					tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, tmp21));
					dynBlV.addScaled(tmp21, bodyB.getInverseMass());
					((RigidBody) bodyB).getAngularVelocity().add(tmp2);
				}
			}
		} else {
			jn = -(1 + Math.max(mA.getRestitution(), mB.getRestitution()))
					* velAlongNormal;
			dynBlV.addScaled(normal, jn);
			if (frictional) {
				relVel = dynBlV;
				velAlongNormal = relVel.dot(normal);
				tangent.setSubtractScaled(relVel, normal, velAlongNormal)
						.norm();
				final float jt = -relVel.dot(tangent);
				if (jt != 0) {
					if (Math.abs(jt) < jn
							* clamp(mA.getStaticFriction(),
									mB.getStaticFriction())) {
						dynBlV.addScaled(tangent, jt);
					} else {
						dynBlV.addScaled(
								tangent,
								-clamp(mA.getDynamicFriction(),
										mB.getDynamicFriction())
										* jn);
					}
				}
			}
		}
	}

	private void dynamicContact(final IUpdateInfo uInfo) {
		final DynamicBody bodyA = (DynamicBody) this.bodyA;
		final DynamicBody bodyB = (DynamicBody) this.bodyB;
		final Vector3f dynAlV = bodyA.getLinearVelocity();
		final Vector3f dynBlV = bodyB.getLinearVelocity();
		rV.setSubtract(dynBlV, dynAlV);
		final float velAlongNormal = rV.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		if ((distance += slop) < 0) {
			rV.setScale(
					normal,
					percent
							* (distance / (bodyA.getInverseMass() + bodyB
									.getInverseMass())));
			bodyA.getPositionCorrection().subtractScaled(rV,
					bodyA.getInverseMass());
			bodyB.getPositionCorrection().addScaled(rV, bodyB.getInverseMass());
		}
		final float inverseInverseMassSum = 1F / (bodyA.getInverseMass() + bodyB
				.getInverseMass());
		final Material mA = bodyA.getMaterial();
		final Material mB = bodyB.getMaterial();
		final float jn = -(1 + Math.max(mA.getRestitution(),
				mB.getRestitution()))
				* velAlongNormal * inverseInverseMassSum;
		dynAlV.subtractScaled(normal, jn * bodyA.getInverseMass());
		dynBlV.addScaled(normal, jn * bodyB.getInverseMass());
		if (velAlongNormal != 0) {
			rV.setSubtract(dynBlV, dynAlV);
			tangent.setSubtractScaled(rV, normal, rV.dot(normal)).norm();

			float jt = rV.dot(tangent);
			if (jt != 0) {
				jt *= inverseInverseMassSum;
				final float clampedStaticFriction = clamp(
						mA.getStaticFriction(), mB.getStaticFriction());
				if (jt < jn * clampedStaticFriction) {
					dynAlV.addScaled(tangent, jt * bodyA.getInverseMass());
					dynBlV.subtractScaled(tangent, jt * bodyB.getInverseMass());
				} else {
					final float fs = clamp(mA.getDynamicFriction(),
							mB.getDynamicFriction())
							* jn;
					dynAlV.addScaled(tangent, fs * bodyA.getInverseMass());
					dynBlV.subtractScaled(tangent, fs * bodyB.getInverseMass());
				}
			}
		}
	}

	private void rigidContact(final IUpdateInfo uInfo) {
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
		if ((distance += slop) < 0) {
			tmp11.setScale(
					normal,
					percent
							* (distance / (bodyA.getInverseMass() + bodyB
									.getInverseMass())));
			bodyA.getPositionCorrection().subtractScaled(tmp11,
					bodyA.getInverseMass());
			bodyB.getPositionCorrection().addScaled(tmp11,
					bodyB.getInverseMass());
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
		float jn = (-(1 + Math.max(mA.getRestitution(), mB.getRestitution())) * velAlongNormal)
				/ (bodyA.getInverseMass() + bodyB.getInverseMass() + tmp11
						.dot(normal));
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
		if (o instanceof ElasticContact) {
			final ElasticContact c = (ElasticContact) o;
			return (bodyA == c.bodyA && bodyB == c.bodyB)
					|| (bodyA == c.bodyB && bodyB == c.bodyA);
		}
		return false;
	}

	@Override
	public int hashCode() {
		return Collidable.unorderdHashCode(bodyA, bodyB);
	}
}
