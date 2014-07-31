package com.brocorporation.gameengine.elements.bodies;

import java.util.ArrayList;
import java.util.List;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.elements.collision.Collidable;
import com.brocorporation.gameengine.elements.collision.CollisionDetection;
import com.brocorporation.gameengine.elements.collision.Constraint;
import com.brocorporation.gameengine.elements.collision.Contact;
import com.brocorporation.gameengine.elements.collision.ElasticContactSolver;
import com.brocorporation.gameengine.elements.collision.GJK;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.elements.collision.MPR;
import com.brocorporation.gameengine.elements.collision.Octree;
import com.brocorporation.gameengine.elements.collision.RaycastHit;
import com.brocorporation.gameengine.elements.collision.SpeculativeContactSolver;
import com.brocorporation.gameengine.elements.collision.Tree;
import com.brocorporation.gameengine.utils.Vector3f;

public class World implements CollisionDetection.BroadphaseCallback {

	public final static float GRAVITY = 9.81F;

	protected final Tree collisionTree = new Octree(0, 0, 0, 25);
	protected final List<DynamicBody> updateList = new ArrayList<DynamicBody>();
	protected final List<Constraint> constraintList = new ArrayList<Constraint>();
	protected Camera activeCamera;
	protected StaticLight activeLight;

	final static Vector3f temp = new Vector3f();
	@Override
	public void broadphaseCollision(final DynamicBody dynBody,
			final StaticBody stcBody, final IUpdateInfo uInfo) {
		final Contact c = Contact.DEFAULT;
		final IShape stcShape = stcBody.getShape();
		final IShape dynShape = dynBody.getShape();
		if (GJK.intersects(c, stcShape, dynShape, 0.02f)) {
			if (c.getDistance() == 0 /*&& MPR.intersects(c, stcShape, dynShape)*/) {
//				ElasticContactSolver.addContact(stcBody, dynBody,
//						c.getNormal(), c.getDistance());
				MPR.getPenetration(c,stcShape,dynShape);
				ElasticContactSolver.addContact(stcBody, dynBody,c.getNormal(), c.getDistance());
			}

		} else {
				SpeculativeContactSolver.addContact(stcBody, dynBody,
						c.getNormal(), c.getDistance());
		}
		
		
		
		
		
	/*	
		if (stcBody instanceof Plane) {
			final Plane plane = (Plane) stcBody;
			final Vector3f n = plane.getNormal();
			if (plane.getSide(dynBody.getPosition()) != Plane.BACK
					&& n.dot(dynBody.getLinearVelocity()) <= 0) {
				final IShape dynShape = dynBody.getShape();
				//final Contact c = Contact.DEFAULT;
				if (GJK.intersects(c, plane.getShape(), dynShape, 0f)) {
//					if (c.getDistance() == 0) {
//						ElasticContactSolver.addContact(plane, dynBody, n,
//								plane.getDistance(dynShape));
						if (MPR.intersects(c, plane.getShape(), dynShape)) {
							ElasticContactSolver.addContact(plane, dynBody,
									c.getNormal(), c.getDistance());
						}
//						MPR.getPenetration(c,plane.getShape(),dynShape);
//						ElasticContactSolver.addContact(plane, dynBody,c.getNormal(), c.getDistance());
					//} else {
					//	ElasticContactSolver.addContact(plane, dynBody,
					//			c.getNormal(), c.getDistance());
					//}
				} else {
					//SpeculativeContactSolver.addContact(plane, dynBody,
					//		c.getNormal(), c.getDistance());
				}

				stcBody.onCollide(dynBody);
				dynBody.onCollide(stcBody);
			}
		} else {
			final IShape dynShape = dynBody.getShape();
			final IShape stcShape = stcBody.getShape();
			final Contact c = Contact.DEFAULT;
			float d = GJK.distance(c, stcShape, dynShape);
			if (d > 0) {
				SpeculativeContactSolver.addContact(stcBody, dynBody,
						c.getNormal(), d);
			} else {
				d = AABB.getDistance(temp, stcShape.getAABB(),
						dynShape.getAABB());
				ElasticContactSolver.addContact(stcBody, dynBody, temp, d);
			}
			stcBody.onCollide(dynBody);
			dynBody.onCollide(stcBody);
		}*/
		stcBody.onCollide(dynBody);
		dynBody.onCollide(stcBody);
	}

	@Override
	public void broadphaseCollision(final DynamicBody dynBody1,
			final DynamicBody dynBody2, final IUpdateInfo uInfo) {
		if ((dynBody1 instanceof Actor && dynBody2 instanceof Item)
				|| (dynBody2 instanceof Actor && dynBody1 instanceof Item)) {
			final Actor actor;
			final Item item;
			if (dynBody1 instanceof Actor) {
				actor = (Actor) dynBody1;
				item = (Item) dynBody2;
			} else {
				actor = (Actor) dynBody2;
				item = (Item) dynBody1;
			}
			final AABB dynAABB = actor.getShape().getAABB();
			if (AABB.intersectsRay(RaycastHit.DEFAULT, item.getShape()
					.getAABB(), dynAABB.getHalfsize(), dynAABB.getPosition(),
					actor.getLinearVelocity(), 0, uInfo.getRate())) {
				dynBody1.onCollide(dynBody2);
				dynBody2.onCollide(dynBody1);
			}
		} else {
			final Contact c = Contact.DEFAULT;
			final IShape dS1 = dynBody1.getShape();
			final IShape dS2 = dynBody2.getShape();
			if (GJK.intersects(c, dS1, dS2, 0.02f)) {
					 if(c.getDistance() == 0 && MPR.intersects(c, dS1, dS2)){
						 ElasticContactSolver.addContact(dynBody1, dynBody2,
								 c.getNormal(), c.getDistance());
//						 MPR.getPenetration(c, dS1, dS2);
//							ElasticContactSolver.addContact(dynBody1, dynBody2,
//									 c.getNormal(), c.getDistance());
					 }
			} else {
				SpeculativeContactSolver.addContact(dynBody1, dynBody2,
						c.getNormal(), c.getDistance());
				
			}

			dynBody1.onCollide(dynBody2);
			dynBody2.onCollide(dynBody1);
		}
	}

	public void update(IUpdateInfo uInfo) {
		for (final DynamicBody dynBody : updateList) {// TODO all bodies
			dynBody.updatePosition(uInfo);
			dynBody.updateBounds();

			final IShape shape = dynBody.getShape();
			if (shape.hasChanged()) {
				collisionTree.hasMoved(shape.getAABB());
			}
			dynBody.prepareUpdatePosition(uInfo);
			dynBody.generateSweptBounds(uInfo);
		}
		for (final Constraint contraint : constraintList) {
			contraint.solve(uInfo);
		}
		CollisionDetection.checkBroadphase(this, updateList, collisionTree,
				uInfo);
		SpeculativeContactSolver.run(uInfo);
		ElasticContactSolver.run(uInfo);

		collisionTree.optimize();
		if (uInfo.isPreRendering()) {
			activeCamera.prepareUpdatePosition(uInfo);
			if (activeCamera instanceof TrackingCamera) {
				final TrackingCamera sc = (TrackingCamera) activeCamera;
				if (sc.followBody != null) {
					CollisionDetection.rayCasting(sc, collisionTree);
				}
			}
			activeCamera.updatePosition(uInfo);
			activeLight.updatePosition(uInfo);
		}
	}

	public void add(final DynamicBody body) {
		updateList.add(body);
		collisionTree.add(body.getShape().getAABB(), body);
	}

	public void add(final Collidable body) {
		collisionTree.add(body.getShape().getAABB(), body);
	}

	public void addConstraint(final Constraint constraint) {
		constraintList.add(constraint);
	}

	public void setActiveCamera(final Camera camera) {
		activeCamera = camera;
	}

	public void setActiveLight(final StaticLight light) {
		activeLight = light;
	}

	public Camera getActiveCamera() {
		return activeCamera;
	}

	public StaticLight getActiveLight() {
		return activeLight;
	}

	public Tree getTree() {
		return collisionTree;
	}
}