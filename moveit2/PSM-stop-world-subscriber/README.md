# stopWorldGeometryMonitor() leaves the PlanningSceneWorld subscription active

`PlanningSceneMonitor::stopWorldGeometryMonitor()` removes both world-geometry subscriptions after one call.

Affected behavior: After one call, CollisionObject is silent but PlanningSceneWorld is still subscribed and replaces the scene. Calling stop a second time removes it. The single-stop arm failed 3/3 and the double-stop control passed 3/3.
