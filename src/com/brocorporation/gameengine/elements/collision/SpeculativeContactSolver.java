package com.brocorporation.gameengine.elements.collision;

import java.util.ArrayDeque;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;

public class SpeculativeContactSolver {

	private final static ArrayDeque<SpeculativeContact> useStack = new ArrayDeque<SpeculativeContact>();
	private final static ArrayDeque<SpeculativeContact> unusedStack = new ArrayDeque<SpeculativeContact>();
	private static int iterations = 2;

	public static SpeculativeContact addContact(final StaticBody stcBody,
			final DynamicBody dynBody, final Contact contact) {
		try {
			final SpeculativeContact c;
			if (unusedStack.isEmpty()) {
				c = new SpeculativeContact(stcBody, dynBody, contact);
			} else {
				c = unusedStack.pop();
				c.reset(stcBody, dynBody, contact);
			}
			useStack.push(c);
			return c;
		} catch (Exception e) {
		}
		return null;
	}

	public static void setIterations(final int pIterations) {
		iterations = pIterations;
	}

	public static void run(final IUpdateInfo uInfo) {
		if (!useStack.isEmpty()) {
			for (int i = 1; i < iterations; i++) {
				for (final SpeculativeContact c : useStack) {
					c.solve(uInfo);
				}
			}
			while (!useStack.isEmpty()) {
				final SpeculativeContact c;
				if ((c = useStack.pop()) != null) {
					c.solve(uInfo);
					c.reset();
					unusedStack.push(c);
				}
			}
		}
	}

	public static void releaseAll() {
		while (!useStack.isEmpty()) {
			final SpeculativeContact c;
			if ((c = useStack.pop()) != null) {
				c.reset();
				unusedStack.push(c);
			}
		}
	}
}
