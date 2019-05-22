#pragma once
#include "Rigid_body.h"


//This class provides an interface for classes that need to interact with a physics simulator, in order to control its behaviour. The methods here 
//will be called by the simulator at certain points (for instance, when determining if collisions need to be performed between two arbitrary objects). 
//It is up to the class that implements this interface to handle each method accordingly.  
class PreCollisionQuery
{
public:
	PreCollisionQuery() = default;

	virtual ~PreCollisionQuery() = default;

	PreCollisionQuery(const PreCollisionQuery& other) = delete;
	PreCollisionQuery(PreCollisionQuery&& other) = delete;
	PreCollisionQuery& operator=(const PreCollisionQuery& other) = delete;
	PreCollisionQuery& operator=(PreCollisionQuery&& other) = delete;

	//	This method returns true if the pair of rigid bodies here should be checked for collisions, false otherwise.
	//	Note that the ground does not have a rigid body associated with it, so if a rigid body is NULL, it means it
	//	corresponds to the ground. The joined variable is set to true if the two bodies are connected by a joint,
	//	false otherwise.
	virtual bool shouldCheckForCollisions(Rigid_body& rb1, Rigid_body& rb2, bool joined);
};
