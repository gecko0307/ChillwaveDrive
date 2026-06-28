module weather;

import std.random;
import dagon;

struct Raindrop
{
    Vector3f position;
    bool active = false;
}

class Rain: Owner, Updateable, Drawable
{
    protected:

    Vector2f[4] vertices;
    Vector2f[4] texcoords;
    uint[3][2] indices;

    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint tbo = 0;
    GLuint eao = 0;
    
    public:
    
    Raindrop[] raindrops;
    Vector3f size = Vector3f(1.0f, 1.0f, 1.0f); //Vector3f(0.02f, 0.5f, 0.02f);
    
    Vector3f spawnPosition = Vector3f(0.0f, 5.0f, 0.0f);
    float interval = 0.01f;
    float speed = 30.0f;
    float rotation = 0.0f;
    
    Vector3f velocity = Vector3f(0.0f, -30.0f, 0.0f);
    
    this(Owner owner)
    {
        super(owner);
        
        vertices[0] = Vector2f(-0.5f, 0.5f);
        vertices[1] = Vector2f(-0.5f, -0.5f);
        vertices[2] = Vector2f(0.5f, -0.5f);
        vertices[3] = Vector2f(0.5f, 0.5f);

        texcoords[0] = Vector2f(0, 0);
        texcoords[1] = Vector2f(0, 1);
        texcoords[2] = Vector2f(1, 1);
        texcoords[3] = Vector2f(1, 0);

        indices[0][0] = 0;
        indices[0][1] = 1;
        indices[0][2] = 2;

        indices[1][0] = 0;
        indices[1][1] = 2;
        indices[1][2] = 3;

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.length * float.sizeof * 2, vertices.ptr, GL_STATIC_DRAW);

        glGenBuffers(1, &tbo);
        glBindBuffer(GL_ARRAY_BUFFER, tbo);
        glBufferData(GL_ARRAY_BUFFER, texcoords.length * float.sizeof * 2, texcoords.ptr, GL_STATIC_DRAW);

        glGenBuffers(1, &eao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eao);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.length * uint.sizeof * 3, indices.ptr, GL_STATIC_DRAW);

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eao);

        glEnableVertexAttribArray(VertexAttrib.Vertices);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(VertexAttrib.Vertices, 2, GL_FLOAT, GL_FALSE, 0, null);

        glEnableVertexAttribArray(VertexAttrib.Texcoords);
        glBindBuffer(GL_ARRAY_BUFFER, tbo);
        glVertexAttribPointer(VertexAttrib.Texcoords, 2, GL_FLOAT, GL_FALSE, 0, null);

        glBindVertexArray(0);
        
        raindrops = New!(Raindrop[])(50);
    }
    
    ~this()
    {
        Delete(raindrops);
        
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &tbo);
        glDeleteBuffers(1, &eao);
    }
    
    float timer = 0.0f;
    
    void update(Time t)
    {
        int numSpawn = 0;
        timer += t.delta;
        if (timer >= interval)
        {
            timer = 0.0f;
            numSpawn = 5;
        }
        
        foreach(ref r; raindrops)
        {
            if (r.active)
            {
                r.position += velocity * t.delta;
                if (r.position.y <= -size.y)
                    r.active = false;
            }
            else
            {
                if (numSpawn > 0)
                {
                    r.active = true;
                    float randomDist = uniform(0.0f, 10.0f);
                    Vector2f randomPosOffset = randomUnitVector2!float * randomDist;
                    r.position = spawnPosition + Vector3f(randomPosOffset.x, uniform(0.0f, 10.0f), randomPosOffset.y);
                    numSpawn--;
                }
            }
        }
    }
    
    void render(GraphicsState* state)
    {
        foreach(ref r; raindrops)
        {
            if (r.active)
                renderRaindrop(r, state);
        }
    }
    
    void renderRaindrop(Raindrop r, GraphicsState* state)
    {
        GraphicsState stateLocal = *state;
        
        Matrix4x4f modelViewMatrix =
            state.viewMatrix *
            translationMatrix(r.position) *
            state.invViewRotationMatrix *
            rotationMatrix(Axis.z, degtorad(rotation)) *
            scaleMatrix(size);
        
        stateLocal.modelViewMatrix = modelViewMatrix;
        stateLocal.prevModelViewMatrix = modelViewMatrix;
        
        if (stateLocal.shader)
        {
            stateLocal.shader.bindParameters(&stateLocal);
        }
        
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, cast(uint)indices.length * 3, GL_UNSIGNED_INT, cast(void*)0);
        glBindVertexArray(0);
        
        if (stateLocal.shader)
        {
            stateLocal.shader.unbindParameters(&stateLocal);
        }
    }
}
