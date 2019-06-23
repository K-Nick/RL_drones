import numpy
import math
 
def magnitude(v):
    """
    Euclidean length of a vector sqrt(x*x+y*y+z*z)
    """
    return math.sqrt(numpy.dot(v,v))
 
def normalise(v):
    """
    returns a normalised copy of the vector v.
    undefined if the length of v is zero
    """
    return v / magnitude(v)