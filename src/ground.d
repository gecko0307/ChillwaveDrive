module ground;

import dlib.core.ownership;
import dlib.container.array;

struct GroundMaterial
{
    float grip = 1.0f;
    float rollingResistanceC0 = 0.001f;
    float rollingResistanceC1 = 0.001f;
    float rollingResistanceC2 = 0.000f;
}

class Ground: Owner
{
    Array!GroundMaterial materials;
    
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
}
