package com.brocorporation.gameengine.elements.collision;

import java.util.ArrayList;
import java.util.List;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.Plane;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.elements.bodies.TrackingCamera;
import com.brocorporation.gameengine.utils.Vector3f;

public class CollisionDetection {

	private final static List<Object> tempList = new ArrayList<Object>(50);

	public static void checkBroadphase(final BroadphaseCallback bCallback,
			final List<DynamicBody> dynamicList, final Tree staticTree,
			final IUpdateInfo uInfo) {
		for (final DynamicBody dynBody1 : dynamicList) {
			if (dynBody1.canActiveCollide()) {
				final AABB dyn1SweptAABB = dynBody1.getSweptAABB();
				final Class<? extends DynamicBody> class1 = dynBody1.getClass();
				staticTree.retrieve(tempList, dyn1SweptAABB);
				for (final Object o : tempList) {
					if (dynBody1 != o) {
						if (o instanceof DynamicBody) {
							final DynamicBody dynBody2 = (DynamicBody) o;
							if (dynBody2.canPassiveCollide(dynBody1)
									&& CollisionFilter.canCollide(class1,
											dynBody2.getClass())
									&& dyn1SweptAABB.intersects(dynBody2
											.getSweptAABB())) {
								bCallback.broadphaseCollision(dynBody1,
										dynBody2, uInfo);
							}
						} else if (o instanceof StaticBody) {
							final StaticBody stcBody = (StaticBody) o;
							if (stcBody.canPassiveCollide(dynBody1)
									&& CollisionFilter.canCollide(class1,
											stcBody.getClass())
									&& dyn1SweptAABB.intersects(stcBody
											.getShape().getAABB())) {
								bCallback.broadphaseCollision(dynBody1,
										stcBody, uInfo);
							}
						}
					}
				}
				tempList.clear();
			}
		}
	}

	private final static Vector3f[] cDirs = { new Vector3f(), new Vector3f(),
			new Vector3f(), new Vector3f() };
	final static Vector3f nearPos = new Vector3f();
	final static Vector3f tmp = new Vector3f();

	public static void rayCasting(final TrackingCamera camera,
			final Tree staticTree) {
		final AABB cameraAABB = camera.getAABB();
		staticTree.retrieve(tempList, cameraAABB);
		if (!tempList.isEmpty()) {
			final float near = camera.getNear();
			final Vector3f side = camera.getSide();
			final Vector3f up = camera.getUp();
			final Vector3f followBodyPosition = camera.getFollowBody()
					.getPosition();
			final Vector3f followDirection = camera.getFinalFollowOffset();
			final Vector3f vd = camera.getNormedViewDirection();
			nearPos.setAdd(followBodyPosition, followDirection).addScaled(vd,
					near);

			cDirs[0].setAddScaled(nearPos, up, camera.getTop());
			cDirs[1].setAddScaled(nearPos, up, camera.getBottom());
			cDirs[2].setAddScaled(nearPos, side, camera.getRight());
			cDirs[3].setAddScaled(nearPos, side, camera.getLeft());

			final Vector3f direction = camera.getViewDirection();
			final float max = 1 - near;
			float min = 0;
			final RaycastHit hit = RaycastHit.DEFAULT;
			for (final Object o : tempList) {
				if (o instanceof Plane) {
					final Plane plane = (Plane) o;
					if (plane.canPassiveCollide()
							&& cameraAABB
									.intersects(plane.getShape().getAABB())) {
						final Vector3f n = plane.getNormal();
						final float nDotVd = n.dot(vd);
						if (nDotVd > 0.001f) {
							for (int i = 0; i < 4; i++) {
								if (plane.intersectsRay(hit, cDirs[i],
										direction, min, max)) {
									min = hit.getScalar();
								}
							}
						}
					}
				}
			}
			tempList.clear();

			if (min != 0 && min != max) {
				followDirection.addScaled(direction, min + near);
			}
		}
	}

	public static interface BroadphaseCallback {
		public void broadphaseCollision(final DynamicBody dynBody,
				final StaticBody stcBody, final IUpdateInfo uInfo);

		public void broadphaseCollision(final DynamicBody dynBody1,
				final DynamicBody dynBody2, final IUpdateInfo uInfo);
	}
}
