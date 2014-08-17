package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class MinkowskiDifference {

	public static void getMaxSupport(final Simplex.Element result,
			final IShape shape1, final Vector3f vertex, final Vector3f direction) {
		result.v.setSubtract(shape1.getMaxAlongDirection(result.pA, direction),
				result.pB.set(vertex));
	}

	public static void getMinSupport(final Simplex.Element result,
			final IShape shape1, final Vector3f vertex, final Vector3f direction) {
		result.v.setSubtract(shape1.getMinAlongDirection(result.pA, direction),
				result.pB.set(vertex));
	}

	public static void getMaxSupport(final Simplex.Element result,
			final IShape shape1, final IShape shape2, final Vector3f direction) {
		result.v.setSubtract(shape1.getMaxAlongDirection(result.pA, direction),
				shape2.getMinAlongDirection(result.pB, direction));
	}

	public static void getMinSupport(final Simplex.Element result,
			final IShape shape1, final IShape shape2, final Vector3f direction) {
		result.v.setSubtract(shape1.getMinAlongDirection(result.pA, direction),
				shape2.getMaxAlongDirection(result.pB, direction));
	}
}
