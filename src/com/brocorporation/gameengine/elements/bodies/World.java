package com.brocorporation.gameengine.elements.bodies;

import java.util.ArrayList;
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

public class World implements CollisionDetection.BroadphaseCallback {

	public final static float GRAVITY = 9.81F;

	protected final Tree collisionTree = new Octree(0, 0, 0, 250);//TODO size
	protected final List<DynamicBody> updateList = new ArrayList<DynamicBody>();
	protected final List<Constraint> constraintList = new ArrayList<Constraint>();
	protected Camera activeCamera;
	protected StaticLight activeLight;
	
	public static boolean debug =false;
	public static int debugid;
	
	@Override
	public void broadphaseCollision(final DynamicBody dynBody,
			final StaticBody stcBody, final IUpdateInfo uInfo) {
		final Contact c = Contact.DEFAULT;
		final IShape stcShape = stcBody.getShape();
		final IShape dynShape = dynBody.getShape();
		if(GJK.distance(c, stcShape, dynShape)==0){
		//if (GJK.intersects(c, stcShape, dynShape, 0.02f)) {
			if (c.getDistance() == 0 || true) {
				// float dd = AABB.getDistance(c.getNormal(),
				// stcBody.getShape().getAABB(), dynBody.getShape().getAABB());
				// if(dd<=0){
				// c.setDistance(dd);
				// ElasticContactSolver.addContact(stcBody, dynBody, c);
				// }
				
				// MPR.relVel.set(dynBody.getLinearVelocity());
				if (MPR.intersects(c, stcShape, dynShape)) {
					if(debug) System.out.println("=================mpr==============");
					if(stcBody instanceof Plane && (((Plane) stcBody).getNormal().y==1 || ((Plane) stcBody).getNormal().y == 0)){
						c.getNormal().set(((Plane) stcBody).getNormal());
						dynShape.getMinAlongDirection(c.getPointB(), c.getNormal());
						c.setDistance(((Plane) stcBody).getDistance(c.getPointB()));
						c.getPointB().setSubtractScaled(c.getPointB(), c.getNormal(), c.getDistance()/2);
						c.getPointA().set(c.getPointB());
					}
					if(c.getDistance() <= 0){
						Manifold m = Manifold.add(stcBody, dynBody, c);
						ElasticContact ec = ElasticContactSolver.addContact(stcBody, dynBody, c);
						ec.manifold = m;
					}
				}
			} else {
				if(debug) System.out.println("=================gjk==============");
				ElasticContactSolver.addContact(stcBody, dynBody, c);
			}	
		} else {
			if(debug) System.out.println("=================spc==============");	
//			Manifold m = Manifold.add(stcBody, dynBody, c);
			
//			SpeculativeContactSolver.addContact(stcBody, dynBody,
//					c);
		}
		if(debug && dynBody.getID()==World.debugid){
			System.out.println(stcBody);
			if(dynBody instanceof RigidBody) {
				MatrixExt.logM("inertia", ((RigidBody) dynBody).getInverseInertiaTensor());
				System.out.println("ang:\t"+((RigidBody) dynBody).getAngularVelocity());
				System.out.println("ang:\t"+((RigidBody) dynBody).getAngularVelocity().length());
				System.out.println("lin:\t"+dynBody.getLinearVelocity());
				System.out.println("lin:\t"+dynBody.getLinearVelocity().length());
			}
			System.out.println("pos:\t"+dynShape.getPosition());
			System.out.println("p A:\t"+c.getPointA());
			System.out.println("nor:\t"+c.getNormal());
			System.out.println("dis:\t"+c.getDistance());
			MyGLSurfaceView.cPSet = true;
			MyGLSurfaceView.cPoint.getPosition().set(c.getPointA());
			MyGLSurfaceView.cPNormal.set(c.getNormal());
			System.out.println("elastic");
		}
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
				if (c.getDistance() == 0) {
					if (MPR.intersects(c, dS1, dS2)) {
						ElasticContactSolver.addContact(dynBody1, dynBody2, c);
					}
				} else {
					ElasticContactSolver.addContact(dynBody1, dynBody2, c);
				}
			} else {
				SpeculativeContactSolver.addContact(dynBody1, dynBody2,
						c);
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
		Manifold.update();
		CollisionDetection.checkBroadphase(this, updateList, collisionTree,
				uInfo);
		ElasticContactSolver.run(uInfo);
		SpeculativeContactSolver.run(uInfo);
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
