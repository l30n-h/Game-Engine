package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public class RigidBody extends DynamicBody {

	protected final Vector3f angularVelocity = new Vector3f();
	protected final Vector3f angularMomentum = new Vector3f();
	protected final float[] inverseInertiaTensor = new float[9];
	protected final Vector3f diagI = new Vector3f();

	protected boolean updateAngularMomantum = false;

	public RigidBody(final IShape shape, final float pMass) {
		super(shape, pMass);
		shape.getInverseInertiaTensor(diagI, inverseMass);
		inverseInertiaTensor[0] = diagI.x;
		inverseInertiaTensor[4] = diagI.y;
		inverseInertiaTensor[8] = diagI.z;
	}

	public void setAngularVelocity(final float degreeX, final float degreeY, final float degreeZ) {
		if (degreeX != 0 || degreeY != 0 || degreeZ != 0) {
			final float s = Quaternion.PIOVER180;
			angularVelocity.set(degreeX * s, degreeY * s, degreeZ * s);
		}
	}

	public Vector3f getAngularVelocity() {
		return angularVelocity;
	}

	public float[] getInverseInertiaTensor() {
		return inverseInertiaTensor;
	}

	public void addImpulse(final float impulseX, final float impulseY, final float impulseZ, final float pX,
			final float pY, final float pZ) {
		super.addImpulse(impulseX, impulseY, impulseZ);
		final Vector3f p = shape.getPosition();
		final float x = pX - p.x;
		final float y = pY - p.y;
		final float z = pZ - p.z;
		addAngularMomentum(y * impulseZ - z * impulseY, z * impulseX - x * impulseZ, x * impulseY - y * impulseX);
	}

	public void addAngularMomentum(Vector3f m) {
		addAngularMomentum(m.x, m.y, m.z);
	}

	public void addAngularMomentum(final float x, final float y, final float z) {
		if (x != 0 || y != 0 || z != 0) {
			angularMomentum.add(x, y, z);
			updateAngularMomantum = true;
		}
	}

	public void clearAngularMomentum() {
		angularMomentum.set(0, 0, 0);
		updateAngularMomantum = false;
	}

	@Override
	public void clearMomenta() {
		super.clearMomenta();
		clearAngularMomentum();
	}

	@Override
	protected void applyMomenta() {
		super.applyMomenta();
		if (updateAngularMomantum) {
			angularVelocity.add(tmp.multiplyM3V(inverseInertiaTensor, 0, angularMomentum));
			clearAngularMomentum();
		}

	}

	static Vector3f tmp = new Vector3f();
	static float[] rot = new float[9];

	// Vector3f oldAngVel = new Vector3f();
	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		super.updatePosition(uInfo);
		if (angularVelocity.x != 0 || angularVelocity.y != 0 || angularVelocity.z != 0) {
			// oldAngVel.add(angularVelocity).scale(0.5f);
			// affineTransform.getOrientation().integrateRotationScaled(
			// oldAngVel, uInfo.getRate());
			// oldAngVel.set(angularVelocity);
			affineTransform.getOrientation().integrateRotationScaled(angularVelocity, uInfo.getRate());
			updateOrientation = true;
		}
		if (updateOrientation) {// TODO only if inertia is needed (Coins not)
			affineTransform.getOrientation().getRotationMatrix3(rot);
			MatrixExt.multiplyM3D3MT3(inverseInertiaTensor, rot, diagI);
		}

	}
}
