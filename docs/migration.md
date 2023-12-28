# Migration Guide

## Version 2.4 to Version 3.0
Box2D version 3.0 is a full rewrite. You can read some background information [here](https://box2d.org/posts/2023/01/starting-box2d-3.0/).

Here are the highlights that affect the API:
- moved from C++ to C
- identifiers (handles) instead of pointers
- multithreading support
- fewer callbacks
- more features (such as capsules and shape casts)

However, the scope of what Box2D does has not changed much. It is still a 2D rigid body engine. It is just faster and more robust (hopefully). And hopefully it is easier to work with and port/wrap for other languages/platforms.

I'm going to describe migration by comparing code snippets between 2.4 and 3.0. These should give you and idea of the sort of transformations you need to make to your code to migrate to v3.0.

### Creating a world
Version 2.4:
```cpp
#include "box2d/box2d.h"
b2Vec2 gravity(0.0f, -10.0f);
b2World world(gravity);
```
Version 3.0:
```c
#include "box2d/box2d.h"
b2Vec2 gravity = {0.0f, -10.0f};
b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.gravity = gravity;
b2WorldId worldId = b2CreateWorld(&worldDef);
```
There is now a required world defition. C does not have constructors, so you need to initialize **ALL** structures that you pass to Box2D. Box2D provides and initialization helper for almost all structures. For example `b2DefaultWorldDef()` is used here to initialize `b2WorldDef`. `b2WorldDef` provides many options, but the defaults are good enough to get going.

In Version 3.0, Box2D objects are generally hidden and you only have an identifier. This keeps the API small. So when you create a world you just get a `b2WorldId` which you should treat as an atomic object, like `int` or `float`. It is small and should be passed by value.

In Version 3.0 there are also no destructors, so you must destroy the world.
```c
b2DestroyWorld(worldId);
worldId = b2_nullWorldId;
```
This destroys all bodies, shapes, and joints as well. This is quicker than destroying them individually. Just like pointers, it is good practice to nullify identifiers. Box2D provides null values for all identifiers and also macros such as `B2_IS_NULL` to test if an identifier is null.

### Creating a body
Version 2.4:
```cpp
b2BodyDef bodyDef;
bodyDef.type = b2_dynamicBody;
bodyDef.position.Set(0.0f, 4.0f);
b2Body* body = world.CreateBody(&bodyDef);
```
Version 3.0:
```c
b2BodyDef bodyDef = b2_defaultBodyDef;
bodyDef.type = b2_dynamicBody;
bodyDef.position = {0.0f, 4.0f};
b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);
```
Body creation is very similar in v3.0. In this case there is a constant definition initializer `b2_defaultBodyDef`. This can help save a bit of typing in some cases. In v3.0 I recommend getting comfortable with curly brace initialization for initializing vectors. There are no member functions in C. Notice that the body is created using a loose function and providing the `b2WorldId` as an argument. Basically what you would expect going from C++ to C.

Destroying a body is also similar.
Version 2.4:
```cpp
world.DestroyBody(body);
body = nullptr;
```
Version 3.0:
```c
b2DestroyBody(bodyId);
bodyId = b2_nullBodyId;
```
Notice there is a little magic here in Version 3.0. `b2BodyId` knows what world it comes from. So you do not need to provide `worldId` when destroying the body. Version 3.0 supports up to 32 worlds. This may increased or be overriden in the future.

There is a significant behavior change with body destruction in Version 3.0.
> Destroying a body no longer destroys the attached joints, it is up to you to destroy them.

Shapes are still destroyed automatically. However, `b2DestructionListener` is gone. This holds to the theme of fewer callbacks.

### Creating a shape
Shape creation has been streamlined in Version 3.0. `b2Fixture` is gone. I feel like it was a confusing concept so I hope you don't miss it.

Version 2.4:
```cpp
b2PolygonShape box;
box.SetAsBox(1.0f, 1.0f);

b2FixtureDef fixtureDef;
fixtureDef.shape = &box;
fixtureDef.density = 1.0f;
fixtureDef.friction = 0.3f;

b2Fixture* fixture = body->CreateFixture(&fixtureDef);
```

Version 3.0:
```c
b2Polygon box = b2MakeBox(1.0f, 1.0f);

b2ShapeDef shapeDef = b2_defaultShapeDef;
shapeDef.density = 1.0f;
shapeDef.friction = 0.3f;

b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &box);
```

So basically v2.4 shapes are no longer shapes, they are _geometry_ with no inheritance (of course). This freed the term _shape_ to be used where _fixture_ was used before. In v3.0 the shape definition is generic and there are different functions for creating each shape type, such as `b2CreateCircleShape` or `b2CreateSegmentShape`.

Again notice the structure initialization with `b2_defaultShapeDef`. Unfortunately we cannot have meaningful definitions with zero initialization. You must initialize your structures.

Destroying shapes is straight forward.

Version 2.4:
```cpp
body->DestroyFixture(fixture);
fixture = nullptr;
```

Version 3.0:
```c
b2DestroyShape(shapeId);
shapeId = b2_nullShapeId;
```

### Chains
In Version 2.4 chains are a type of shape. In Version 3.0 they are a separate concept. This lead to significant simplifications internally. In Version 2.4 all shapes had to support the notion of child shapes. This is gone.

Version 2.4:
```cpp
b2Vec2 points[5];
points[0].Set(-8.0f, 6.0f);
points[1].Set(-8.0f, 20.0f);
points[2].Set(8.0f, 20.0f);
points[3].Set(8.0f, 6.0f);
points[4].Set(0.0f, -2.0f);

b2ChainShape chain;
chain.CreateLoop(points, 5);
b2FixtureDef fixtureDef;
fixtureDef.shape = &chain;
b2Fixture* chainFixture = body->CreateFixture(&fixtureDef);
```

Version 3.0:
```c
b2Vec2 points[5] = {
    {-8.0f, 6.0f},
    {-8.0f, 20.0f},
    {8.0f, 20.0f},
    {8.0f, 6.0f},
    {0.0f, -2.0f}
};

b2ChainDef chainDef = b2_defaultChainDef;
chainDef.points = points;
chainDef.count = 5;
chainDef.loop = true;
b2ChainId chainId = b2CreateChain(bodyId, &chainDef);
```

Since chains are their own concept now, they get their own identifier, `b2ChainId`. You can view chains as macro objects, they create many `b2SmoothSegment` shapes internally. Normally you don't interact with these. However they are returned from queries. I may need to write an API to allow you to get the `b2ChainId` for a smooth segment that you get from a query.

> DO NOT destroy or modify a `b2SmoothSegment` that belongs to a chain shape directly

### Creating a joint

### Contact data

### Sensors

### Queries


### Library configuration
