module ground;

import dlib.core.ownership;
import dlib.container.array;

struct GroundMaterial
{
    float grip;
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
