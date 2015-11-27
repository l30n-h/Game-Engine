package com.brocorporation.gameengine.elements.bodies;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.MyGLSurfaceView;
import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.elements.collision.Collidable;
import com.brocorporation.gameengine.elements.collision.CollisionDetection;
import com.brocorporation.gameengine.elements.collision.Constraint;
import com.brocorporation.gameengine.elements.collision.Contact;
import com.brocorporation.gameengine.elements.collision.ElasticContact;
import com.brocorporation.gameengine.elements.collision.ElasticContactSolver;
import com.brocorporation.gameengine.elements.collision.GJK;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.elements.collision.MPR;
import com.brocorporation.gameengine.elements.collision.Manifold;
import com.brocorporation.gameengine.elements.collision.Octree;
import com.brocorporation.gameengine.elements.collision.RaycastHit;
import com.brocorporation.gameengine.elements.collision.SpeculativeContactSolver;
import com.brocorporation.gameengine.elements.collision.Tree;
import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Vector3f;

public class World implements CollisionDetection.BroadphaseCallback {

	public final static float GRAVITY = 9.81F;

	protected final Tree collisionTree = new Octree(0, 0, 0, 250);// TODO size
	protected final List<DynamicBody> updateList = new ArrayList<DynamicBody>();
	protected final List<Constraint> constraintList = new ArrayList<Constraint>();
	protected Camera activeCamera;
	protected StaticLight activeLight;

	public static boolean debug = false;
	public static int debugid;

	public final static Vector3f temp = new Vector3f();

	@Override
	public void broadphaseCollision(final DynamicBody dynBody, final StaticBody stcBody, final IUpdateInfo uInfo) {
		final Contact c = Contact.DEFAULT;
		final IShape stcShape = stcBody.getShape();
		final IShape dynShape = dynBody.getShape();
		// if(GJK.distance(c, stcShape, dynShape)==0){
		// if(true){
		if (GJK.intersects(c, stcShape, dynShape, 0.02f)) {
			if (c.getDistance() == 0) {
				// MPR.DIR.set(dynBody.getLinearVelocity());
				if (stcBody instanceof Plane) {
					temp.setSubtract(stcBody.getPosition(), dynBody.getPosition()).add(dynBody.getLinearVelocity());
					if (temp.dot(((Plane) stcBody).getNormal()) > 0)
						return;
					MPR.DIR.setInvert(((Plane) stcBody).getNormal());
					MPR.setDir(true);
				}
				if (MPR.intersects(c, stcShape, dynShape)) {
					if (debug)
						System.out.println("=================mpr==============");
					if (c.getDistance() <= 0) {
						Manifold.add(stcBody, dynBody, c);
					}
				}
			} else {
				if (debug)
					System.out.println("=================gjk==============");
				Manifold.add(stcBody, dynBody, c);
			}
		} else {
			if (debug)
				System.out.println("=================spc==============");
			// Manifold m = Manifold.add(stcBody, dynBody, c);

			// SpeculativeContactSolver.addContact(stcBody, dynBody,
			// c);
		}
		if (debug && dynBody.getID() == World.debugid) {
			System.out.println(stcBody);
			if (dynBody instanceof RigidBody) {
				MatrixExt.logM("inertia", ((RigidBody) dynBody).getInverseInertiaTensor());
				System.out.println("ang:\t" + ((RigidBody) dynBody).getAngularVelocity());
				System.out.println("ang:\t" + ((RigidBody) dynBody).getAngularVelocity().length());
				System.out.println("lin:\t" + dynBody.getLinearVelocity());
				System.out.println("lin:\t" + dynBody.getLinearVelocity().length());
			}
			System.out.println("pos:\t" + dynShape.getPosition());
			System.out.println("p A:\t" + c.getPointA());
			System.out.println("nor:\t" + c.getNormal());
			System.out.println("dis:\t" + c.getDistance());
			MyGLSurfaceView.cPSet = true;
			MyGLSurfaceView.cPoint.getPosition().set(c.getPointA());
			MyGLSurfaceView.cPNormal.set(c.getNormal());
			System.out.println("elastic");
		}
		stcBody.onCollide(dynBody);
		dynBody.onCollide(stcBody);
	}

	@Override
	public void broadphaseCollision(final DynamicBody dynBody1, final DynamicBody dynBody2, final IUpdateInfo uInfo) {
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
			if (AABB.intersectsRay(RaycastHit.DEFAULT, item.getShape().getAABB(), dynAABB.getHalfsize(),
					dynAABB.getPosition(), actor.getLinearVelocity(), 0, uInfo.getRate())) {
				dynBody1.onCollide(dynBody2);
				dynBody2.onCollide(dynBody1);
			}
		} else {
			final Contact c = Contact.DEFAULT;
			final IShape dS1 = dynBody1.getShape();
			final IShape dS2 = dynBody2.getShape();
			if (GJK.distance(c, dS1, dS2) == 0) {
				if (true) {
					// if (GJK.intersects(c, dS1, dS2, 0.02f)) {
					// if (c.getDistance() == 0) {
					if (MPR.intersects(c, dS1, dS2)) {
						Manifold.add(dynBody1, dynBody2, c);
					}
				} else {
					Manifold.add(dynBody1, dynBody2, c);
				}

			} else {
				// SpeculativeContactSolver.addContact(dynBody1, dynBody2,
				// c);
			}
			dynBody1.onCollide(dynBody2);
			dynBody2.onCollide(dynBody1);
		}
	}

	public void update(IUpdateInfo uInfo) {
		for (final DynamicBody dynBody : updateList) {// TODO all bodies
			dynBody.prepareUpdatePosition(uInfo);

			dynBody.updatePosition(uInfo);
			dynBody.updateBounds();
			final IShape shape = dynBody.getShape();
			if (shape.hasChanged()) {
				collisionTree.hasMoved(shape.getAABB());
			}

			// dynBody.prepareUpdatePosition(uInfo);
			// dynBody.generateSweptBounds(uInfo);
			dynBody.generateSweptBoundsBackwards(uInfo);
		}
		for (final Constraint contraint : constraintList) {
			contraint.solve(uInfo);
		}
		Manifold.update();
		CollisionDetection.checkBroadphase(this, updateList, collisionTree, uInfo);
		Collection<Manifold> c = Manifold.manifolds.values();
		for (Manifold m : c) {
			for (int i = 0; i < m.size(); i++) {
				if (m.getContact(i).getDistance() <= 0) {
					ElasticContact e = ElasticContactSolver.addContact(m.getBodyA(), (DynamicBody) m.getBodyB(),
							m.getContact(i));
				}
				// if(m.size()==4){
				// e.setWarmStarting(m.j);
				// }
			}
		}
		ElasticContactSolver.run(uInfo);
		SpeculativeContactSolver.run(uInfo);
		collisionTree.optimize();
		updatePreRendering(uInfo);
	}

	public void updatePreRendering(IUpdateInfo uInfo) {
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

	public void update2(IUpdateInfo uInfo) {
		CollisionDetection.checkBroadphase(this, updateList, collisionTree, uInfo);
		Collection<Manifold> c = Manifold.manifolds.values();
		for (Manifold m : c) {
			for (int i = 0; i < m.size(); i++) {
				ElasticContact e = ElasticContactSolver.addContact(m.getBodyA(), (DynamicBody) m.getBodyB(),
						m.getContact(i));
				// if(m.size()==4){
				// e.setWarmStarting(m.j);
				// }
			}
		}
		ElasticContactSolver.run(uInfo);
		SpeculativeContactSolver.run(uInfo);
		for (final DynamicBody dynBody : updateList) {// TODO all bodies
			dynBody.prepareUpdatePosition(uInfo);
			dynBody.generateSweptBounds(uInfo);
		}
		for (final Constraint contraint : constraintList) {
			contraint.solve(uInfo);
		}

		for (final DynamicBody dynBody : updateList) {
			dynBody.updatePosition(uInfo);
			dynBody.updateBounds();
			final IShape shape = dynBody.getShape();
			if (shape.hasChanged()) {
				collisionTree.hasMoved(shape.getAABB());
			}
		}
		collisionTree.optimize();
		Manifold.update();
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
