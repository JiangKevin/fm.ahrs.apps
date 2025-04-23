#pragma once

#include "BaseApp.h"

namespace Urho3D
{
    class Node;
    class Scene;
}

class Vehicle;
//

class RaycastVehicleDemo : public BaseApp
{
    URHO3D_OBJECT( RaycastVehicleDemo, BaseApp );
public:
    /// Construct.
    explicit RaycastVehicleDemo( Context* context );

    /// Setup after engine initialization and before running the main loop.
    void Start() override;
private:
    /// Create static scene content.
    void CreateScene();

    /// Create the vehicle.
    void CreateVehicle();

    /// Construct an instruction text to the UI.
    void CreateInstructions();

    /// Subscribe to necessary events.
    void SubscribeToEvents();

    /// Handle application update. Set controls to vehicle.
    void HandleUpdate( StringHash eventType, VariantMap& eventData );

    /// Handle application post-update. Update camera position after vehicle has moved.
    void HandlePostUpdate( StringHash eventType, VariantMap& eventData );

    /// The controllable vehicle component.
    WeakPtr< Vehicle > vehicle_;
};
