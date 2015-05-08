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
	protected final float[] defI = new float[9];// TODO replace by Vector after
												// rotation matrix

	protected boolean updateAngularMomantum = false;

	public RigidBody(final IShape shape, final float pMass) {
		super(shape, pMass);
		shape.getInverseInertiaTensor(inverseInertiaTensor, inverseMass);
		MatrixExt.setM4(defI, inverseInertiaTensor);
	}

	public void setAngularVelocity(final float degreeX, final float degreeY,
			final float degreeZ) {
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
	protected void applyMomenta() {// TODO eigentlich müsste *0,5 sein
		if (updateLinearMomentum) {
			linearVelocity.addScaled(linearMomentum, inverseMass);
			if (updateAngularMomantum) {
				angularVelocity.add(tmp.multiplyM3V(inverseInertiaTensor, 0,
						angularMomentum));
			}
			clearMomenta();
		}
	}

	static Vector3f tmp = new Vector3f();
	static float[] rot = new float[9];
	static float[] tra = new float[9];

	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		super.updatePosition(uInfo);
		if (angularVelocity.x != 0 || angularVelocity.y != 0
				|| angularVelocity.z != 0) {
			affineTransform.getOrientation().addRotationScaled(angularVelocity,
					uInfo.getRate());
			updateOrientation = true;
		}
		if (updateOrientation) {// TODO only if inertia is needed (Coins not)
			affineTransform.getOrientation().getRotationMatrix3(rot);
			MatrixExt.transposeM3(tra, rot);
			MatrixExt.multiplyM3M3(inverseInertiaTensor, 0, rot, 0, defI, 0);
			MatrixExt.multiplyM3M3(inverseInertiaTensor, 0,
					inverseInertiaTensor, 0, tra, 0);// (RxI)xT
		}
	}
}
