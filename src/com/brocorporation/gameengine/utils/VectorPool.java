package com.brocorporation.gameengine.utils;

import java.util.HashSet;
import java.util.Set;
import java.util.Stack;

public class VectorPool {

	private final static Set<Vector3f> useForCycleSet = new HashSet<Vector3f>();
	private final static Set<Vector3f> useSet = new HashSet<Vector3f>();
	private final static Stack<Vector3f> unusedV3Stack = new Stack<Vector3f>();

	public static Vector3f getVectorV3(boolean forCycle) {
		final Vector3f vector;
		if (unusedV3Stack.isEmpty()) {
			vector = new Vector3f();
		} else {
			vector = unusedV3Stack.pop();
			vector.set(0, 0, 0);
		}
		(forCycle ? useForCycleSet : useSet).add(vector);
		return vector;
	}

	public static void release(final Vector3f vector) {
		if (vector != null) {
			if (useSet.remove(vector)) {
				unusedV3Stack.push(vector);
			}
		}
	}

	public static void releaseAll(boolean onlyCycle) {
		for (final Vector3f vector : useForCycleSet) {
			if (useForCycleSet.remove(vector)) {
				unusedV3Stack.push(vector);
			}
		}
		if (!onlyCycle) {
			for (final Vector3f vector : useSet) {
				release(vector);
			}
		}
	}
}
