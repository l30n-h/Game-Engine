package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.Plane;
import com.brocorporation.gameengine.elements.bodies.RigidBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class ElasticContact extends Constraint {

	protected final static Vector3f tangent = new Vector3f();
	protected final static Vector3f rV = new Vector3f();

	protected final static float percent = 0.8f;
	protected final static float slop = 0.05f;
	// protected final static float percent = 1.0f;
	// protected final static float slop = 0.0f;

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

	static boolean frictional = false;// TODO

	private void staticContact(final IUpdateInfo uInfo) {
		final Vector3f dynBlV = dynB.getLinearVelocity();
		boolean isRigidB = dynB instanceof RigidBody;
		Vector3f relVel = isRigidB ? rV.setAdd(dynBlV,
				rV.setCross(((RigidBody) dynB).getAngularVelocity(), pointB))
				: dynBlV;
		float velAlongNormal = relVel.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			dynB.getPositionCorrection().addScaled(normal, percent * distance);
		}
		final Material mA = stcA.getMaterial();
		final Material mB = dynB.getMaterial();
		final float jn;
		if (isRigidB) {
			final float[] itB = ((RigidBody) dynB).getInverseInertiaTensor();
			tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, normal));
			tmp11.setCross(tmp2, pointB);
			jn = (-(1 + Math.max(mA.getRestitution(), mB.getRestitution())) * velAlongNormal)
					/ (dynB.getInverseMass() + tmp11.dot(normal));
			dynBlV.addScaled(normal, dynB.getInverseMass() * jn);
			((RigidBody) dynB).getAngularVelocity().addScaled(tmp2, jn);
			System.out.println(dynBlV);
			System.out.println(((RigidBody) dynB).getAngularVelocity());
			System.out.println();
			if (frictional) {
				relVel = rV.setAdd(dynBlV, rV.setCross(
						((RigidBody) dynB).getAngularVelocity(), pointB));
				velAlongNormal = relVel.dot(normal);
				tangent.setSubtractScaled(relVel, normal, velAlongNormal)
						.norm();
				tmp2.multiplyM3V(itB, 0, tmp2.setCross(pointB, tangent));
				tmp11.setCross(tmp2, pointB);
				float jt = -relVel.dot(tangent);
				if (jt != 0) {
					jt /= (dynB.getInverseMass() + tmp11.dot(tangent));
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
					dynBlV.addScaled(tmp21, dynB.getInverseMass());
					((RigidBody) dynB).getAngularVelocity().add(tmp2);
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
		final Vector3f dynAlV = dynA.getLinearVelocity();
		final Vector3f dynBlV = dynB.getLinearVelocity();
		rV.setSubtract(dynBlV, dynAlV);
		final float velAlongNormal = rV.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
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
		final float inverseInverseMassSum = 1F / (dynA.getInverseMass() + dynB
				.getInverseMass());
		final Material mA = dynA.getMaterial();
		final Material mB = dynB.getMaterial();
		final float jn = -(1 + Math.max(mA.getRestitution(),
				mB.getRestitution()))
				* velAlongNormal * inverseInverseMassSum;
		dynAlV.subtractScaled(normal, jn * dynA.getInverseMass());
		dynBlV.addScaled(normal, jn * dynB.getInverseMass());
		if (velAlongNormal != 0) {
			rV.setSubtract(dynBlV, dynAlV);
			tangent.setSubtractScaled(rV, normal, rV.dot(normal)).norm();

			float jt = rV.dot(tangent);
			if (jt != 0) {
				jt *= inverseInverseMassSum;
				final float clampedStaticFriction = clamp(
						mA.getStaticFriction(), mB.getStaticFriction());
				if (jt < jn * clampedStaticFriction) {
					dynAlV.addScaled(tangent, jt * dynA.getInverseMass());
					dynBlV.subtractScaled(tangent, jt * dynB.getInverseMass());
				} else {
					final float fs = clamp(mA.getDynamicFriction(),
							mB.getDynamicFriction())
							* jn;
					dynAlV.addScaled(tangent, fs * dynA.getInverseMass());
					dynBlV.subtractScaled(tangent, fs * dynB.getInverseMass());
				}
			}
		}
	}

	private void rigidContact(final IUpdateInfo uInfo) {
		final Vector3f dynAlV = dynA.getLinearVelocity();
		final Vector3f dynBlV = dynB.getLinearVelocity();
		boolean isRigidA = dynA instanceof RigidBody;
		boolean isRigidB = dynB instanceof RigidBody;
		rV.setSubtract(
				isRigidB ? tmp2.setAdd(dynBlV, tmp2.setCross(
						((RigidBody) dynB).getAngularVelocity(), pointB))
						: dynBlV,
				isRigidA ? tmp1.setAdd(dynAlV, tmp1.setCross(
						((RigidBody) dynA).getAngularVelocity(), pointA))
						: dynAlV);
		float velAlongNormal = rV.dot(normal);
		if (velAlongNormal >= 0) {
			return;
		}
		if (distance > 0) {
			distance = 0;
		} else if ((distance += slop) < 0) {
			tmp11.setScale(
					normal,
					percent
							* (distance / (dynA.getInverseMass() + dynB
									.getInverseMass())));
			dynA.getPositionCorrection().subtractScaled(tmp11,
					dynA.getInverseMass());
			dynB.getPositionCorrection()
					.addScaled(tmp11, dynB.getInverseMass());
		}
		final Material mA = dynA.getMaterial();
		final Material mB = dynB.getMaterial();

		if (isRigidA) {
			tmp1.multiplyM3V(((RigidBody) dynA).getInverseInertiaTensor(), 0,
					tmp1.setCross(pointA, normal));
			tmp11.setCross(tmp1, pointA);
		} else {
			tmp11.set(0, 0, 0);
		}
		if (isRigidB) {
			tmp2.multiplyM3V(((RigidBody) dynB).getInverseInertiaTensor(), 0,
					tmp2.setCross(pointB, normal));
			tmp21.setCross(tmp2, pointB);
			tmp11.add(tmp21);
		}
		float jn = (-(1 + Math.max(mA.getRestitution(), mB.getRestitution())) * velAlongNormal)
				/ (dynA.getInverseMass() + dynB.getInverseMass() + tmp11
						.dot(normal));
		dynAlV.subtractScaled(normal, dynA.getInverseMass() * jn);
		dynBlV.addScaled(normal, dynB.getInverseMass() * jn);
		if (isRigidA)
			((RigidBody) dynA).getAngularVelocity().subtractScaled(tmp1, jn);
		if (isRigidB)
			((RigidBody) dynB).getAngularVelocity().addScaled(tmp2, jn);

		if (frictional) {
			rV.setSubtract(
					isRigidB ? tmp2.setAdd(dynBlV, tmp2.setCross(
							((RigidBody) dynB).getAngularVelocity(), pointB))
							: dynBlV,
					isRigidA ? tmp1.setAdd(dynAlV, tmp1.setCross(
							((RigidBody) dynA).getAngularVelocity(), pointA))
							: dynAlV);
			velAlongNormal = rV.dot(normal);
			tangent.setSubtractScaled(rV, normal, velAlongNormal).norm();
			float jt = -rV.dot(tangent);
			if (jt != 0) {
				if (isRigidA) {
					tmp1.multiplyM3V(
							((RigidBody) dynA).getInverseInertiaTensor(), 0,
							tmp1.setCross(pointA, tangent));
					tmp11.setCross(tmp1, pointA);
				} else {
					tmp11.set(0, 0, 0);
				}
				if (isRigidB) {
					tmp2.multiplyM3V(
							((RigidBody) dynB).getInverseInertiaTensor(), 0,
							tmp2.setCross(pointB, tangent));
					tmp21.setCross(tmp2, pointB);
					tmp11.add(tmp21);
				}
				jt /= (dynA.getInverseMass() + dynB.getInverseMass() + tmp11
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
				dynAlV.subtractScaled(tmp21, dynA.getInverseMass());
				dynBlV.addScaled(tmp21, dynB.getInverseMass());
				if (isRigidA) {
					tmp1.multiplyM3V(
							((RigidBody) dynA).getInverseInertiaTensor(), 0,
							tmp1.setCross(pointA, tmp21));
					((RigidBody) dynA).getAngularVelocity().subtract(tmp1);
				}
				if (isRigidB) {
					tmp2.multiplyM3V(
							((RigidBody) dynB).getInverseInertiaTensor(), 0,
							tmp2.setCross(pointB, tmp21));
					((RigidBody) dynB).getAngularVelocity().add(tmp2);
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
