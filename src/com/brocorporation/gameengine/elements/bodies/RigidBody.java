package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public class RigidBody extends DynamicBody {

	protected final Vector3f angularVelocity = new Vector3f();
	protected final Vector3f angularMomentum = new Vector3f();
	protected final Vector3f inverseInertiaTensor = new Vector3f();

	protected boolean updateAngularMomantum = false;
	protected boolean addAngularVelocity = false;

	public RigidBody(final IShape shape, final float pMass) {
		super(shape, pMass);
	}

	public void setAngularVelocity(final float degreeX, final float degreeY,
			final float degreeZ) {
		if (degreeX != 0 || degreeY != 0 || degreeZ != 0) {
			final float s = Quaternion.PIOVER180;
			angularVelocity.set(degreeX * s, degreeY * s, degreeZ * s);
			addAngularVelocity = true;
		} else {
			addAngularVelocity = false;
		}
	}

	public void addImpulse(final float impulseX, final float impulseY,
			final float impulseZ, final float pX, final float pY, final float pZ) {
		super.addImpulse(impulseX, impulseY, impulseZ);
		final Vector3f p = shape.getPosition();
		final float x = pX - p.x;
		final float y = pY - p.y;
		final float z = pZ - p.z;
		angularMomentum.add(y * impulseZ - z * impulseY, z * impulseX - x
				* impulseZ, x * impulseY - y * impulseX);
		updateAngularMomantum = true;
	}

	@Override
	public void clearMomenta() {
		super.clearMomenta();
		updateAngularMomantum = false;
	}

	@Override
	protected void applyMomenta() {
		if (updateLinearMomentum) {
			linearVelocity.addScaled(linearMomentum, inverseMass);
			if (updateAngularMomantum) {
				angularVelocity.add(angularMomentum.x * inverseInertiaTensor.x,
						angularMomentum.y * inverseInertiaTensor.y,
						angularMomentum.z * inverseInertiaTensor.z);
				addAngularVelocity = angularVelocity.x != 0
						|| angularVelocity.y != 0 || angularVelocity.z == 0;
			}
			clearMomenta();
		}
	}

	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		super.updatePosition(uInfo);
		if (addAngularVelocity) {
			affineTransform.getOrientation().addRotationScaled(angularVelocity,
					uInfo.getRate());
			updateOrientation = true;
		}
		// InertiaTensor.getCuboid(inertiaTensor, mass, halfsize);TODO
	}
}
