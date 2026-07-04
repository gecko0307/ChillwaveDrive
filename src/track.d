module track;

import dlib.core.ownership;
import dlib.container.array;
import dlib.math.vector;

struct GroundMaterial
{
    float grip = 1.0f;
    float rollingResistanceC0 = 0.001f;
    float rollingResistanceC1 = 0.001f;
    float rollingResistanceC2 = 0.000f;
}

class Track: Owner
{
    ///
    Vector3f[] waypoints;
    
    ///
    Array!GroundMaterial materials;
    
    ///
    uint numLaps = 3;
    
    this(Owner owner)
    {
        super(owner);
    }
    
    ~this()
    {
        materials.free();
    }
    
    void addMaterial(GroundMaterial m)
    {
        materials.append(m);
    }
    
    size_t getNearestSegmentIndex(size_t currentSegmentIndex, Vector3f carPosition)
    {
        // Update the current segment (search for the closest point to the car within a small range).
        // Limit the search to the vicinity of the old index so it doesn't jump to the finish line
        size_t searchStart = (currentSegmentIndex > 2) ? currentSegmentIndex - 2 : 0;
        size_t searchEnd = currentSegmentIndex + 5;
        if (searchEnd >= waypoints.length) searchEnd = waypoints.length;
        
        float minD = float.max;
        size_t bestIdx = currentSegmentIndex;

        for (size_t i = searchStart; i < searchEnd; ++i)
        {
            float distSq = (carPosition.x - waypoints[i].x)^^2 + (carPosition.z - waypoints[i].z)^^2;
            if (distSq < minD)
            {
                minD = distSq;
                bestIdx = i;
            }
        }
        
        // Segment index is the index of the starting point of the segment
        if (bestIdx < waypoints.length - 1)
            return bestIdx;
        else
            return 0; // Looping the index when crossing the finish line
    }
}
