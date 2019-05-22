#include "PreCollisionQuery.h"
#include "ArticulatedRigidBody.h"

bool PreCollisionQuery::shouldCheckForCollisions(Rigid_body& rb1, Rigid_body& rb2, bool joined)
{
	//we do not want to check the objects for collisions if they are connected by a joint (joint limits should be used for that).
	if (joined)
		return false;

	//don't allow collisions between static things
	if (rb1.is_locked() && rb2.is_locked())
		return false;

	//don't allow collisions between object on the same chain
	if (rb1.is_articulated() && rb2.is_articulated())
	{
		auto children_arb = dynamic_cast<Articulated_rigid_body&>(rb1).get_children_arbs(true);
		for (auto& arb : children_arb)
		{
			if (arb == &rb2)
			{
				return false;
			}
		}
		children_arb = dynamic_cast<Articulated_rigid_body&>(rb2).get_children_arbs(true);
		for (auto& arb : children_arb)
		{
			if (arb == &rb1)
			{
				return false;
			}
		}
	}
	return true;
}
