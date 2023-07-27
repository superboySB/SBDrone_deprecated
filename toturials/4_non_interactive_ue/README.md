# Non-interactive Unreal Engine Custom Environments

We provide the environment presented within the paper to allow others to validate our approach. However, to create a custom environment, we recomend you follow the following steps to prevent agent interaction.

## Remove ego-perspective rendering of other quadrotors
To make the quadrotor invisible in the scene, change the 'Hidden in Scene Capture' to True. This will make it invisible to other drones but the spectator actor can still see it. Go to details, then rendering, this will show the setting 'ACtor Hidden In Game'.

![](./images/MakeActorHidden.png)

![](./images/MakeActorHiddenZoom.png)

## Remove Collision boxes from all agents within the environment
We need to specifically remove agent-agent interaction while also enabling environment interaction. Hence we need to define all components of the quadrotor blueprint 'BP_FlyingPawn' as 'Pawn' and ignore any overlaps that occour between this group. To do this, we modify the collision response within the agent blueprint.

There are five components to change within the 'BP_FlyingPawn' blueprint: BodyMesh, Prop3, Prop2, Prop1, Prop0. For all of these, go to collisions, then change the collision presents to custom. Thange the Object Type to 'Pawn' and then in 'Object Responses' change the Pawn to Ignore as shown bellow.

![](./images/CollisionPresets.png)

Now to remove collisions between 'Pawns', we need to ignore the event 'ActorBeginOverlap' which we can do using a Blueprint Event Graph. Add the following event graph to 'BP_FlyingPawn'.

![](./images/IgnoreCollisionBP.png)

Agents will interact with the environment without interacting with each other.