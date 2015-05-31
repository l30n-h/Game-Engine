package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.VectorPool;

public class DistanceConstraint extends Constraint {

	protected float distance;

	public DistanceConstraint(final StaticBody a, final DynamicBody b,
			final float d) throws Exception {
		setBodies(a, b);
		distance = d;
	}

	public void setDistance(final float d) {
		distance = d;
	}

	public float getDistance() {
		return distance;
	}

	@Override
	public void solve(final IUpdateInfo uInfo) {
		final DynamicBody bodyB = (DynamicBody) this.bodyB;
		if (bodyA.isStatic()) {
			final Vector3f dV = VectorPool.getVectorV3(false);
			dV.setSubtract(bodyB.getPosition(), bodyA.getPosition());
			final float currentDistance = dV.length();
			dV.scale(1F / currentDistance);
			final float relVel = bodyB.getLinearVelocity().dot(dV);
			final float relDist = currentDistance - distance;
			final float remove = relVel + relDist * uInfo.getInverseRate();
			bodyB.getLinearVelocity().subtractScaled(dV, remove);
			VectorPool.release(dV);
		} else {
			final DynamicBody bodyA = (DynamicBody) this.bodyA;
			final Vector3f dV = VectorPool.getVectorV3(false);
			final Vector3f vV = VectorPool.getVectorV3(false);
			dV.setSubtract(bodyB.getPosition(), bodyA.getPosition());
			final float currentDistance = dV.length();
			dV.scale(1F / currentDistance);
			vV.setSubtract(bodyB.getLinearVelocity(), bodyA.getLinearVelocity());
			final float relVel = vV.dot(dV);
			final float relDist = currentDistance - distance;
			final float remove = relVel + relDist * uInfo.getInverseRate();
			dV.scale(remove / (bodyA.getInverseMass() + bodyB.getInverseMass()));
			bodyB.getLinearVelocity()
					.subtractScaled(dV, bodyB.getInverseMass());
			bodyA.getLinearVelocity().addScaled(dV, bodyA.getInverseMass());
			VectorPool.release(dV);
			VectorPool.release(vV);
		}
	}
}
