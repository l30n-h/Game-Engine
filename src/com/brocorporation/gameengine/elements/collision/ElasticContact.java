package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.RigidBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class ElasticContact extends Constraint {

	protected final static float percent = 0.8f;
	protected final static float slop = 0.04f;
	// protected final static float percent = 1.0f;
	// protected final static float slop = 0.0f;

	protected Manifold.ManifoldContact contact;
	protected Vector3f normal, tangent1, tangent2;
	protected final Vector3f axn = new Vector3f();
	protected final Vector3f bxn = new Vector3f();
	protected final Vector3f Iaxn = new Vector3f();
	protected final Vector3f Ibxn = new Vector3f();
	protected final Vector3f axt1 = new Vector3f();
	protected final Vector3f bxt1 = new Vector3f();
	protected final Vector3f Iaxt1 = new Vector3f();
	protected final Vector3f Ibxt1 = new Vector3f();
	protected final Vector3f axt2 = new Vector3f();
	protected final Vector3f bxt2 = new Vector3f();
	protected final Vector3f Iaxt2 = new Vector3f();
	protected final Vector3f Ibxt2 = new Vector3f();
	protected final Vector3f pointA = new Vector3f();
	protected final Vector3f pointB = new Vector3f();
	protected float normalImpulseSum, t1ImpulseSum, t2ImpulseSum;
	protected float den, b, dent1, dent2, stcfrt, dynfrt;
	protected boolean hasFriction;

	static Vector3f tmp1 = new Vector3f();// TODO

	public ElasticContact(final StaticBody a, final DynamicBody b,
			final Manifold.ManifoldContact contact) throws Exception {
		reset(a, b, contact);
	}

	public void reset(final StaticBody a, final DynamicBody b,
			final Manifold.ManifoldContact contact) throws Exception {
		setBodies(a, b);
		this.contact = contact;
		normal = contact.getNormal();
		pointA.setSubtract(contact.getPointA(), a.getPosition());
		pointB.setSubtract(contact.getPointB(), b.getPosition());

		// tmp1.setSubtractScaled(contact.getPointB(), normal, distance/2);
		// pointA.setSubtract(tmp1, a.getPosition());
		// pointB.setSubtract(tmp1, b.getPosition());

		if (bodyA instanceof DynamicBody) {
			if (normal.y < -0.25f) {
				((DynamicBody) bodyA).isOnGround(true);
			}
		}
		if (normal.y > 0.25f) {
			((DynamicBody) bodyB).isOnGround(true);
		}
	}

	@Override
	public void reset() {
		super.reset();
		normalImpulseSum = 0;
		t1ImpulseSum = 0;
		t2ImpulseSum = 0;
	}

	@Override
	public void solve(final IUpdateInfo uInfo) {
		if (bodyA.isStatic()) {
			staticlagrangian(uInfo);
		} else {
			lagrangian(uInfo);
		}
	}

	public void prepare(final IUpdateInfo uInfo) {
		if (bodyB instanceof DynamicBody) {
			tmp1.set(((DynamicBody) bodyB).getLinearVelocity());
		} else
			tmp1.set(0, 0, 0);
		if (bodyA instanceof DynamicBody) {
			tmp1.subtract(((DynamicBody) bodyA).getLinearVelocity());
		}
		hasFriction = !contact.getManifold().isFrictionless();
		if (hasFriction) {
			stcfrt = contact.getManifold().getStaticFriction();
			dynfrt = contact.getManifold().getDynamicFriction();
			tangent1 = contact.getTangent1();
			tangent2 = contact.getTangent2();
		}
		float vel = tmp1.dot(normal);
		dent1 = dent2 = den = bodyA.getInverseMass() + bodyB.getInverseMass();
		if (bodyA instanceof RigidBody) {
			RigidBody bodyA = (RigidBody) this.bodyA;
			axn.setCross(pointA, normal);
			Iaxn.multiplyM3V(bodyA.getInverseInertiaTensor(), 0, axn);
			den += axn.dot(Iaxn);
			vel -= axn.dot(bodyA.getAngularVelocity());
			if (hasFriction) {
				axt1.setCross(pointA, tangent1);
				Iaxt1.multiplyM3V(bodyA.getInverseInertiaTensor(), 0, axt1);
				dent1 += axt1.dot(Iaxt1);
				axt2.setCross(pointA, tangent2);
				Iaxt2.multiplyM3V(bodyA.getInverseInertiaTensor(), 0, axt2);
				dent2 += axt2.dot(Iaxt2);
			}
		}
		if (bodyB instanceof RigidBody) {
			RigidBody bodyB = (RigidBody) this.bodyB;
			bxn.setCross(pointB, normal);
			Ibxn.multiplyM3V(bodyB.getInverseInertiaTensor(), 0, bxn);
			den += bxn.dot(Ibxn);
			vel += bxn.dot(bodyB.getAngularVelocity());
			if (hasFriction) {
				bxt1.setCross(pointB, tangent1);
				Ibxt1.multiplyM3V(bodyB.getInverseInertiaTensor(), 0, bxt1);
				dent1 += bxt1.dot(Ibxt1);
				bxt2.setCross(pointB, tangent2);
				Ibxt2.multiplyM3V(bodyB.getInverseInertiaTensor(), 0, bxt2);
				dent2 += bxt2.dot(Ibxt2);
			}
		}
		float eslop = 0.25f;
		b = contact.getManifold().getRestitution() * Math.min(vel + eslop, 0);
		b += 0.1f * uInfo.getInverseRate()
				* Math.min(contact.getDistance() + slop, 0);
	}

	public void staticlagrangian(final IUpdateInfo uInfo) {
		RigidBody bodyB = (RigidBody) this.bodyB;

		float num = bodyB.getLinearVelocity().dot(normal)
				+ bxn.dot(bodyB.getAngularVelocity());
		float lagrangian = -(num + b) / den;

		float lastImp = normalImpulseSum;
		if ((normalImpulseSum += lagrangian) < 0)
			normalImpulseSum = 0;
		float jn = normalImpulseSum - lastImp;

		bodyB.getLinearVelocity()
				.addScaled(normal, jn * bodyB.getInverseMass());
		bodyB.getAngularVelocity().addScaled(Ibxn, jn);

		if (hasFriction) {
			final float si = stcfrt * normalImpulseSum;
			final float di = dynfrt * normalImpulseSum;
			float num1 = bodyB.getLinearVelocity().dot(tangent1)
					+ bxt1.dot(bodyB.getAngularVelocity());
			float num2 = bodyB.getLinearVelocity().dot(tangent2)
					+ bxt2.dot(bodyB.getAngularVelocity());

			lagrangian = -(num1) / dent1;
			lastImp = t1ImpulseSum;
			if (Math.abs(t1ImpulseSum += lagrangian) > si)
				t1ImpulseSum = Math.signum(t1ImpulseSum) * di;
			float jt = t1ImpulseSum - lastImp;

			bodyB.getLinearVelocity().addScaled(tangent1,
					jt * bodyB.getInverseMass());
			bodyB.getAngularVelocity().addScaled(Ibxt1, jt);

			lagrangian = -(num2) / dent2;
			lastImp = t2ImpulseSum;
			if (Math.abs(t2ImpulseSum += lagrangian) > si)
				t2ImpulseSum = Math.signum(t2ImpulseSum) * di;
			jt = t2ImpulseSum - lastImp;

			bodyB.getLinearVelocity().addScaled(tangent2,
					jt * bodyB.getInverseMass());
			bodyB.getAngularVelocity().addScaled(Ibxt2, jt);
		}
	}

	public void lagrangian(final IUpdateInfo uInfo) {
		RigidBody bodyA = (RigidBody) this.bodyA;
		RigidBody bodyB = (RigidBody) this.bodyB;

		float num = tmp1.setSubtract(bodyB.getLinearVelocity(),
				bodyA.getLinearVelocity()).dot(normal)
				+ bxn.dot(bodyB.getAngularVelocity())
				- axn.dot(bodyA.getAngularVelocity());
		float lagrangian = -(num + b) / den;

		float lastImp = normalImpulseSum;
		if ((normalImpulseSum += lagrangian) < 0)
			normalImpulseSum = 0;
		float jn = normalImpulseSum - lastImp;

		bodyA.getLinearVelocity().subtractScaled(normal,
				jn * bodyA.getInverseMass());
		bodyA.getAngularVelocity().subtractScaled(Iaxn, jn);
		bodyB.getLinearVelocity()
				.addScaled(normal, jn * bodyB.getInverseMass());
		bodyB.getAngularVelocity().addScaled(Ibxn, jn);

		if (hasFriction) {
			final float si = stcfrt * normalImpulseSum;
			final float di = dynfrt * normalImpulseSum;
			float num1 = tmp1.setSubtract(bodyB.getLinearVelocity(),
					bodyA.getLinearVelocity()).dot(tangent1)
					+ bxt1.dot(bodyB.getAngularVelocity())
					- axt1.dot(bodyA.getAngularVelocity());
			float num2 = tmp1.dot(tangent2)
					+ bxt2.dot(bodyB.getAngularVelocity())
					- axt2.dot(bodyA.getAngularVelocity());

			lagrangian = -(num1) / dent1;
			lastImp = t1ImpulseSum;
			if (Math.abs(t1ImpulseSum += lagrangian) > si)
				t1ImpulseSum = Math.signum(t1ImpulseSum) * di;
			float jt = t1ImpulseSum - lastImp;

			bodyA.getLinearVelocity().subtractScaled(tangent1,
					jt * bodyA.getInverseMass());
			bodyA.getAngularVelocity().subtractScaled(Iaxt1, jt);
			bodyB.getLinearVelocity().addScaled(tangent1,
					jt * bodyB.getInverseMass());
			bodyB.getAngularVelocity().addScaled(Ibxt1, jt);

			lagrangian = -(num2) / dent2;
			lastImp = t2ImpulseSum;
			if (Math.abs(t2ImpulseSum += lagrangian) > si)
				t2ImpulseSum = Math.signum(t2ImpulseSum) * di;
			jt = t2ImpulseSum - lastImp;

			bodyA.getLinearVelocity().subtractScaled(tangent2,
					jt * bodyA.getInverseMass());
			bodyA.getAngularVelocity().subtractScaled(Iaxt2, jt);
			bodyB.getLinearVelocity().addScaled(tangent2,
					jt * bodyB.getInverseMass());
			bodyB.getAngularVelocity().addScaled(Ibxt2, jt);
		}
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
