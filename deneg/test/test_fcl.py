import fcl
import numpy as np

g1 = fcl.Sphere(1)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)
t1_final = fcl.Transform(np.array([1.0, 0.0, 0.0]))

g2 = fcl.Sphere(1)
t2 = fcl.Transform(np.array([5.0, 0.0, 0.0]))
o2 = fcl.CollisionObject(g2, t2)
t2_final = fcl.Transform(np.array([-2.0, 0.0, 0.0]))

request = fcl.ContinuousCollisionRequest()
result = fcl.ContinuousCollisionResult()

ret = fcl.continuousCollide(
    o1, t1_final, o2, t2_final, request, result)

print(ret)
print(result.is_collide)
print(result.time_of_contact)
