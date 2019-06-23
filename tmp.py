from numpy import *
import math
from phyutils import *
 
# defines an infinite plane passing through a point with
# an arbitrary normal vector

class HalfPlane:
    def __init__(self, origin, normal):
        self.origin = origin
        self.normal = normalise(normal)
 
class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
 
def circleIntersectsHalfPlane(halfPlane,circle):
    vectorOC = circle.center-halfPlane.origin
    vectorOCPlaneNormalComponent = dot(vectorOC,halfPlane.normal)
    return (vectorOCPlaneNormalComponent - circle.radius) < 0
 
def testCircleHalfPlaneIntersection():
    def createCircle(y):
        return Circle( array([0.0,y]), 1.0 )
    
    # define the ground halfplane to pass through the
    # origin, 'upwards' is positive y
    ground = HalfPlane( array([0,0]), array([0,1]) )
    
    # clearly in
    assert( circleIntersectsHalfPlane(ground, createCircle(2.0))   == False)
 
    # boundary conditions
    assert( circleIntersectsHalfPlane(ground, createCircle(1.001)) == False)
    assert( circleIntersectsHalfPlane(ground, createCircle(1.0))   )
    assert( circleIntersectsHalfPlane(ground, createCircle(0.009)) == True)
 
    # clearly out
    assert( circleIntersectsHalfPlane(ground, createCircle(0.0))   == True)
 
    # potential corner case
    assert( circleIntersectsHalfPlane(ground, createCircle(-1.0))  == True)
    
if __name__ == "__main__":
    testCircleHalfPlaneIntersection()